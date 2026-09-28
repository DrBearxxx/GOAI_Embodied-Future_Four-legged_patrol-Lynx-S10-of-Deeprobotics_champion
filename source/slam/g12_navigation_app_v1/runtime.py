"""Orin map/odom/navigation service; physical control only via explicit gateway option."""
import argparse
from collections import deque
import fcntl
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from logging.handlers import RotatingFileHandler
import multiprocessing as mp
import os
from pathlib import Path
import queue
import secrets
import threading
import time
import uuid
import numpy as np
from paths import ROOT, BASE, ASSETS, INDOOR, verify
from protocol import sign, seed_request, Tickets
from worker import worker_main
from local_odometry import frontend_main
from continuous_pose import ContinuousPose
from automatic_localization import load_config as load_auto_config
from lightning_odometry import LightningOdometry, load_config, TOPIC
from navigation import Navigator
from gateway_bridge import GatewayBridge
from mission import Mission
from backend.flight_log import FlightLog,navigation_sample
from indoor.pipeline import pack_cloud, snapshot_ingest, latest_put
from indoor.continuity import use_redundancy
from native_nav.robust_timing import use_slewed_clocks
from s10nav.engine import Engine
from s10nav.messages import convert
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class MapService(Node):
    def __init__(self,gateway_url=None,gateway_key=None):
        super().__init__('wym_g12_map_localizer')
        self.asset_id = verify()
        display = json.loads((ROOT/'android/assets/map.json').read_text())
        if display['asset_id'] != self.asset_id: raise ValueError('DISPLAY_MAP_MISMATCH')
        self.map_id = display['map_id']; self.bounds = display['bounds']
        self.anchors = np.array(display['anchors'])
        self.cfg = json.loads((BASE/'config.json').read_text())
        self.ingest = use_redundancy(use_slewed_clocks(Engine(ASSETS,self.cfg,None)))
        self.auto_config=load_auto_config(ROOT)
        self.fusion = ContinuousPose(self.auto_config);self.auto_trace_key=None
        self.odometry_config = load_config(ROOT)
        self.odom_backend = self.odometry_config['backend']
        self.lightning = LightningOdometry(self.odometry_config) if self.odom_backend == 'lightning' else None
        self.frontend_latest = {}; self.frontend_drops = 0
        self.applied_relocalization=None;self.local_reset_id=None
        self.trace = FlightLog(ROOT/'logs/navigation-trace.jsonl')
        self.bridge = GatewayBridge(gateway_url,gateway_key,trace=self.trace)
        self.mission = Mission({'full':json.loads((ASSETS/'route.json').read_text()),
                                'indoor':json.loads((INDOOR/'assets/route.json').read_text())},
                                self.bridge,ROOT/'private/presets.json')
        self.navigator = self.mission.navigator
        resume_path=ROOT/'private/navigation-resume.json'
        if resume_path.exists():
            saved=json.loads(resume_path.read_text())
            if saved['asset_id']!=self.asset_id:raise ValueError('RESUME_MAP_MISMATCH')
            self.mission.restore_paused(saved)
            resume_path.replace(ROOT/'private/navigation-resume.used.json')
            self.trace.emit('paused_progress_restored',route_id=self.navigator.route_id,target_index=self.navigator.target,
                reached=self.navigator.reached,skipped=self.navigator.skipped)
        self.navigation_requests = {}; self.last_publish = 0.
        self.K = None; self.sensors = {}; self.rejects = {}; self.drops = 0
        self.lock = threading.RLock(); self.tickets = Tickets(); self.jobs = {}
        self.navigation_tickets=Tickets(.8);self.keepalive_tickets=Tickets(1.);self.operator_tickets=Tickets(.4)
        self.state = {}; self.latest = None; self.track = deque(maxlen=1500)
        self.last_trace = 0.; self.last_log = 0.; self.run_id = str(uuid.uuid4())
        self.ctx = mp.get_context('spawn')
        self.start_worker()
        self.start_frontend()
        qos = lambda n: QoSProfile(depth=n,reliability=ReliabilityPolicy.BEST_EFFORT)
        topics = [(f'/wym/slam/{s}/imu',Imu) for s in ('front','rear')]
        topics += [(f'/wym/slam/{s}/points',PointCloud2) for s in ('front','rear')]
        topics += [('/wym/slam/insight9/imu',Imu),('/wym/slam/insight9/odometry',Odometry),
                   ('/wym/slam/insight9/depth/image_rect_raw',Image)]
        self.subs = [self.create_subscription(c,t,lambda m,i,t=t:self.input(t,m,i),qos(64 if c==Imu else 2)) for t,c in topics]
        self.subs.append(self.create_subscription(CameraInfo,'/wym/slam/insight9/infra1/camera_info',self.camera_info,qos(1)))
        if self.lightning is not None:
            self.subs.append(self.create_subscription(String,self.odometry_config.get('topic',TOPIC),self.lightning_input,20))
        self.pub = self.create_publisher(String,'/wym/g12/localization',1)
        self.nav_pub = self.create_publisher(String,'/wym/g12/navigation',1)
        self.create_timer(.05,self.tick)

    def start_frontend(self):
        self.local_inboxes=[self.ctx.Queue(1),self.ctx.Queue(1)]
        self.local_outbox=self.ctx.Queue(3);self.local_stop=self.ctx.Event()
        self.local_commands=self.ctx.Queue(1);self.local_reset_id=None
        self.local_heartbeat=self.ctx.Value('d',time.monotonic(),lock=False)
        self.local_worker=self.ctx.Process(target=frontend_main,args=(self.local_inboxes,self.local_outbox,
            self.local_stop,self.local_heartbeat,self.local_commands,self.odom_backend=='lightning'),daemon=True)
        self.local_worker.start()

    def stop_frontend(self):
        self.local_stop.set();self.local_worker.join(2)
        if self.local_worker.is_alive():self.local_worker.terminate();self.local_worker.join(2)
        if self.local_worker.is_alive():self.local_worker.kill();self.local_worker.join(1)
        for q in [*self.local_inboxes,self.local_outbox,self.local_commands]:q.cancel_join_thread();q.close()

    def sync_relocalization(self,job):
        request_id=job.get('request_id')
        if job.get('state')!='SUCCEEDED' or not request_id or request_id==self.applied_relocalization:return False
        # Global alignment changes the map-to-odom transform, never the local
        # odometry frame/history needed to time-align the delayed observation.
        self.applied_relocalization=request_id
        self.trace.emit('operator_alignment_committed',request_id=request_id)
        return True

    def install_frontend(self,incoming):
        if incoming.get('reset_request_id')!=self.local_reset_id:return False
        self.frontend_latest=incoming
        if getattr(self,'odom_backend','gicp')=='gicp':self.fusion.odometry(incoming.get('odom'))
        return True

    def active_odometry(self):
        return self.lightning.latest if self.lightning is not None else self.frontend_latest.get('odom')

    def restart_frontend(self):
        if self.odom_backend=='gicp':
            self.mission.pause('LOCAL_ODOMETRY_RESTARTED');self.bridge.stop();self.fusion=ContinuousPose(self.auto_config)
        self.stop_frontend();self.start_frontend();self.frontend_latest={}

    def lightning_input(self,msg):
        now=time.monotonic()
        try:
            payload=json.loads(msg.data)
            if not isinstance(payload,dict):raise ValueError('INVALID_LIGHTNING_MESSAGE')
            # Same native->monotonic mapping as map registration and IMU ingest.
            if payload.get('sensor')!=self.lightning.sensor:raise ValueError('LIGHTNING_SOURCE_MISMATCH')
            stamp=int(payload['stamp_ns'])*1e-9
            measured=self.ingest.clocks[self.lightning.sensor].map_stamp(stamp,now)
            previous=self.lightning.epoch
            sample=self.lightning.accept(payload,measured,now)
            if sample:
                self.rejects.pop('lightning',None)
                self.fusion.odometry(sample)
                if previous!=sample['epoch']:
                    self.trace.emit('odometry_epoch',source='lightning',previous=previous,epoch=sample['epoch'])
            else:self.rejects['lightning']=self.lightning.reason
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            self.rejects['lightning']=str(exc)[:160]

    def start_worker(self):
        self.inboxes = [self.ctx.Queue(1), self.ctx.Queue(1)]
        self.outbox = self.ctx.Queue(2); self.commands = self.ctx.Queue(4)
        self.stop = self.ctx.Event(); self.heartbeat = self.ctx.Value('d',time.monotonic(),lock=False)
        self.transaction = self.ctx.Array('c',80)
        self.worker = self.ctx.Process(target=worker_main,args=(self.inboxes,self.outbox,self.commands,self.stop,self.heartbeat,self.transaction),daemon=True)
        self.worker.start()
        self.worker_started = time.monotonic()
        self.auto_trace_key=None

    def stop_worker(self):
        self.stop.set(); self.worker.join(2)
        if self.worker.is_alive(): self.worker.terminate(); self.worker.join(2)
        if self.worker.is_alive(): self.worker.kill(); self.worker.join(1)
        for q in [*self.inboxes,self.outbox,self.commands]: q.cancel_join_thread(); q.close()

    def camera_info(self,m):
        K=np.asarray(m.k).reshape(3,3)
        if np.isfinite(K).all() and K[0,0]>0 and K[1,1]>0: self.K=K

    def input(self,topic,msg,info):
        now=time.monotonic(); receipt=info.get('received_timestamp',0)
        lag=(time.time_ns()-receipt)*1e-9 if receipt else float('inf')
        if not -.02<=lag<=.35:
            self.rejects[topic]='DDS_RECEIPT_STALE';return
        received=now-lag
        try:
            if topic.endswith('/points'):
                side=0 if '/front/' in topic else 1
                item=pack_cloud(topic,msg,received,snapshot_ingest(self.ingest,now))
                item['local_motion']=self.active_odometry()
                if self.auto_config.get('enabled') and self.fusion.offset is not None:
                    item['map_alignment']=dict(T=self.fusion.offset.tolist(),generation=[self.fusion.epoch,self.fusion.revision])
                self.drops+=latest_put(self.inboxes[side],item)
                self.frontend_drops+=latest_put(self.local_inboxes[side],item)
            else:
                e=convert(topic,msg,received,self.K)
                if e:self.ingest.event(e,now)
            self.sensors[topic]=received
        except (ValueError,TypeError,KeyError,IndexError,OverflowError,AttributeError) as exc:
            self.rejects[topic]=str(exc)[:160]

    def tick(self):
        now=time.monotonic()
        if not self.worker.is_alive() or now-self.heartbeat.value>45:
            with self.lock:
                for job in self.jobs.values():
                    if job['state'] in ('QUEUED','SEARCHING'):job.update(state='FAILED',reason='WORKER_RESTARTED_RESELECT_POINT')
                self.stop_worker(); self.start_worker()
                self.latest=None
            logging.warning('relocalization worker restarted; local odometry unchanged')
        if not self.local_worker.is_alive() or now-self.local_heartbeat.value>10:
            with self.lock:
                self.restart_frontend()
            logging.warning('perception frontend restarted; odometry backend=%s',self.odom_backend)
        while True:
            try:
                incoming=self.local_outbox.get_nowait()
                self.install_frontend(incoming)
            except queue.Empty:break
        while True:
            try:self.latest=self.outbox.get_nowait()
            except queue.Empty:break
        latest=self.latest or {}; snapshot=latest.get('snapshot'); job=latest.get('job',{})
        self.sync_relocalization(job)
        if snapshot and job.get('state')=='SUCCEEDED':
            anchor=snapshot.get('anchor')
            if anchor:
                # UUID survives matcher restarts; only a distinct operator
                # request can change an existing alignment.
                anchor=dict(anchor,generation=[job['request_id'],job.get('epoch',0)])
                self.fusion.map_update(anchor)
        pending=latest.get('pending',False)
        automatic=latest.get('automatic')
        if automatic and not pending:
            applied=self.fusion.automatic_update(automatic,now)
            key=(automatic.get('seq'),automatic.get('measurement_mono'))
            if key!=self.auto_trace_key:
                self.auto_trace_key=key
                self.trace.emit('automatic_map_observation',observation=automatic,applied=applied,
                    correction_status=dict(self.fusion.auto_status))
        solution=self.fusion.estimate(now,self.ingest.motion)
        mode=solution['mode'] if solution else 'WAIT_SEED'
        valid=bool(solution and mode in ('TRACKING','DEGRADED','ODOM_BRIDGE','PREDICT_ONLY') and not pending)
        pose=solution['pose'] if solution else None
        generation=solution.get('generation') if solution else None
        if pose and valid and now-self.last_trace>.5:
            self.track.append(dict(xyz=pose[:3],epoch=generation,t=now));self.last_trace=now
        with self.lock:
            self.bridge.poll(now)
            nav=self.mission.step(solution,self.frontend_latest.get('records'),now,pending)
            control=self.bridge.state()
            self.trace.emit('navigation_tick',**navigation_sample(nav,solution,self.frontend_latest.get('records'),control,
                dict(axes=self.mission.rc,sample_seq=self.mission.rc_seq,received_mono=self.mission.rc_at)),
                local_odometry=self.active_odometry(),local_reason=self.lightning.reason if self.lightning else self.frontend_latest.get('reason'),
                odometry_backend=self.odom_backend,
                fit_diagnostics=self.frontend_latest.get('fit_diagnostics'),
                registration_recording=self.frontend_latest.get('registration_recording'),
                scan_recording=self.frontend_latest.get('scan_recording'),
                obstacle_recording=self.frontend_latest.get('obstacle_recording'),
                map_anchor=snapshot.get('anchor') if snapshot else None,
                map_metrics=snapshot.get('metrics') if snapshot else None)
        state=dict(schema='s10.g12.map.v1',map_id=self.map_id,asset_id=self.asset_id,
            frame='joint_map',run_id=self.run_id,pose=pose,valid=valid,
            mode='RELOCALIZING' if pending else mode,solution=solution,
            health=snapshot.get('health') if snapshot else None,
            metrics=snapshot.get('metrics') if snapshot else None,
            sensors={t:round(now-v,3) for t,v in self.sensors.items()},input_rejects=dict(self.rejects),
            queue_drops=self.drops,worker_age_s=max(0.,time.monotonic()-self.heartbeat.value),
            track=list(self.track),job=job,motion_commands_sent=self.bridge.command_count,
            local_odometry=self.lightning.status(now) if self.lightning else {k:v for k,v in self.frontend_latest.items() if k!='records'},
            odometry_backend=self.odom_backend,
            map_association_enabled=bool(self.auto_config.get('enabled')),
            automatic_localization=dict(self.fusion.auto_status),automatic_recording=latest.get('automatic_recording'),
            perception=self.frontend_latest.get('records',{}),frontend_drops=self.frontend_drops,
            navigation=nav,control=control,mission=self.mission.state(now,include_catalog=False),
            recording=self.trace.status(),
            navigation_version='1.33-auto-map',
            scope='ORIN_UNIFIED_NAVIGATION_V1' if self.bridge.client else 'ORIN_UNIFIED_SHADOW',absolute_accuracy_verified=False)
        with self.lock:
            if job.get('request_id') in self.jobs:
                # A stale outbox cannot regress an already terminal request.
                stored=self.jobs[job['request_id']]
                if stored['state'] in ('QUEUED','SEARCHING'):
                    if stored.get('state')!=job.get('state'):logging.info('job %s',json.dumps(job,ensure_ascii=False))
                    self.jobs[job['request_id']]=job.copy()
            self.state=state
            self.state_time=now
        msg=String();msg.data=json.dumps(nav,ensure_ascii=False,allow_nan=False);self.nav_pub.publish(msg)
        if now-self.last_publish>=.1:
            msg=String();msg.data=json.dumps(state,ensure_ascii=False,allow_nan=False);self.pub.publish(msg)
            self.last_publish=now
        if now-self.last_log>10:
            logging.info('status mode=%s valid=%s pose=%s sensors=%d drops=%d job=%s',state['mode'],valid,pose,len(self.sensors),self.drops,job.get('state'))
            self.last_log=now

    def request(self,path,body):
        now=time.monotonic()
        with self.lock:
            if path=='/state':
                if self.keepalive_tickets.consume(body.get('navigation_ticket'),now):
                    self.mission.keepalive(body.get('navigation_run_id'),now)
                state=dict(self.state)
                if not body.get('include_catalog') and 'mission' in state:
                    state['mission']={k:v for k,v in state['mission'].items() if k not in ('catalog','preview','current_route')}
                if body.get('include_catalog'):state['mission']=self.mission.state()
                if body.get('calibration_route'):state['calibration']=self.mission.plans.calibration.state(body['calibration_route'])
                if body.get('recording_preview'):state['route_recording']=self.mission.recorder.state(True)
                state['server_age_s']=now-getattr(self,'state_time',now)
                if state['server_age_s']>1:state['valid']=False;state['mode']='STALE'
                if self.jobs:state['job']=next(reversed(self.jobs.values())).copy()
                return {**state,'ticket':self.tickets.issue(now),'command_ticket':self.navigation_tickets.issue(now),
                        'navigation_ticket':self.keepalive_tickets.issue(now),'operator_ticket':self.operator_tickets.issue(now)}
            if path=='/navigation':
                rid=str(uuid.UUID(body['request_id']))
                if rid in self.navigation_requests:return self.navigation_requests[rid]
                if not self.navigation_tickets.consume(body.get('ticket'),now):raise ValueError('EXPIRED_NAVIGATION_TICKET_REFRESH_STATE')
                if body.get('map_id')!=self.map_id:raise ValueError('MAP_MISMATCH')
                action=body.get('action');solution=self.state.get('solution')
                if action in ('start','shadow','replan','apply_plan'):
                    if now-getattr(self,'state_time',-1e9)>.3:raise ValueError('LOCALIZATION_SERVICE_STALE')
                    if any(j['state'] in ('QUEUED','SEARCHING') for j in self.jobs.values()):raise ValueError('RELOCALIZATION_BUSY')
                extra=self.mission.command(body,solution,now)
                if hasattr(self,'trace'):
                    self.trace.emit('operator_action',action=action,request_id=rid,run_id=self.navigator.run_id,
                        route=self.navigator.route_id,policy=self.mission.desired_policy(),entry=extra.get('entry'),
                        policy_mode=self.mission.policy_mode(),progress=extra.get('progress'),cruise_mps=self.navigator.cruise_mps,
                        route_snapshot=self.navigator.route if action in ('start','shadow','apply_plan','set_progress','calibration_apply') else None,
                        calibration=extra.get('calibration'),calibration_capture=extra.get('capture'),
                        route_recording=extra.get('route_recording'),temporary_waypoints=extra.get('temporary_waypoints'))
                reply=dict(request_id=rid,action=action,accepted=True,control=self.bridge.state(),**extra)
                self.navigation_requests[rid]=reply
                while len(self.navigation_requests)>128:self.navigation_requests.pop(next(iter(self.navigation_requests)))
                logging.info('navigation action %s',json.dumps(reply))
                return reply
            if path=='/operator':
                if not self.operator_tickets.consume(body.get('ticket'),now):raise ValueError('EXPIRED_OPERATOR_TICKET')
                self.mission.operator(body,now)
                return {'accepted':True,'operator_ticket':self.operator_tickets.issue(now)}
            if path=='/stop':
                self.mission.pause('OPERATOR_STOP')
                if hasattr(self,'trace'):self.trace.emit('operator_stop',run_id=self.navigator.run_id)
                return {'accepted':True}
            if path=='/relocalize':
                command=seed_request(body,self.map_id,self.bounds)
                rid=command['request_id']
                if rid in self.jobs:return {'job':self.jobs[rid],'duplicate':True}
                if not self.tickets.consume(body.get('ticket'),now):raise ValueError('EXPIRED_TICKET_REFRESH_STATE')
                if any(j['state'] in ('QUEUED','SEARCHING') for j in self.jobs.values()):raise ValueError('RELOCALIZATION_BUSY')
                xyz=np.array(command['xyz']);d=self.anchors-xyz
                if not np.any((np.linalg.norm(d[:,:2],axis=1)<=command['radius']) & (abs(d[:,2])<=.8)):
                    raise ValueError('NO_REFERENCE_KEYFRAMES_NEAR_POINT_CHECK_HEIGHT')
                solution=self.state.get('solution') or {}
                if solution.get('pose') and solution.get('map_to_odom'):
                    command.update(heading_prior=solution['pose'][3],map_to_odom=solution['map_to_odom'],
                        heading_odom_epoch=solution['generation'][0])
                self.transaction.value=('ACTIVE:'+rid).encode()
                self.mission.pause('RELOCALIZATION_REQUESTED');self.bridge.stop()
                self.commands.put_nowait(command)
                job=dict(state='QUEUED',request_id=rid,xyz=command['xyz'],radius=command['radius'],reason='等待传感器几何验证')
                self.jobs[rid]=job
                while len(self.jobs)>64:self.jobs.pop(next(iter(self.jobs)))
                logging.info('seed request %s',json.dumps(job))
                return {'job':job}
            if path=='/cancel':
                if not self.tickets.consume(body.get('ticket'),now):raise ValueError('EXPIRED_TICKET_REFRESH_STATE')
                rid=body.get('request_id')
                if rid not in self.jobs:raise ValueError('UNKNOWN_JOB')
                if self.jobs[rid]['state'] in ('QUEUED','SEARCHING'):
                    with self.transaction.get_lock():
                        if self.transaction.value.decode()=='COMMITTED:'+rid:
                            return {'job':self.jobs[rid], 'note':'ALREADY_COMMITTED_REFRESH_STATE'}
                        self.transaction.value=('CANCELLED:'+rid).encode()
                        self.commands.put_nowait({'cancel':rid})
                        self.jobs[rid].update(state='CANCELLED',reason='已取消请求')
                return {'job':self.jobs[rid]}
            raise ValueError('UNKNOWN_ENDPOINT')

def handler(service,key):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):super().setup();self.connection.settimeout(3)
        def log_message(self,*args):pass
        def do_POST(self):
            nonce=None
            try:
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<=8192:raise ValueError('BAD_LENGTH')
                raw=self.rfile.read(n)
                if not hmac.compare_digest(sign(key,self.path,raw),self.headers.get('X-S10-MAC','')):
                    self.send_error(403);return
                body=json.loads(raw)
                if not isinstance(body,dict):raise ValueError('EXPECTED_OBJECT')
                nonce=body.get('nonce')
                if not isinstance(nonce,str):raise ValueError('INVALID_NONCE')
                uuid.UUID(nonce)
                value={'ok':True,**service.request(self.path,body)}
            except (ValueError,KeyError,TypeError,queue.Full) as exc:value={'ok':False,'error':str(exc)}
            except (ConnectionError,TimeoutError):return
            raw=json.dumps({**value,'nonce':nonce},ensure_ascii=False,allow_nan=False,separators=(',',':')).encode()
            try:
                self.send_response(200);self.send_header('Content-Type','application/json; charset=utf-8')
                self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store')
                self.send_header('X-S10-MAC',sign(key,self.path,raw));self.end_headers();self.wfile.write(raw)
            except (ConnectionError,TimeoutError):pass
    return Handler

def main():
    p=argparse.ArgumentParser();p.add_argument('--bind',default='10.21.33.102');p.add_argument('--port',type=int,default=18894)
    p.add_argument('--gateway-url',help='local unified gateway, e.g. http://127.0.0.1:18895')
    a=p.parse_args()
    (ROOT/'logs').mkdir(exist_ok=True);(ROOT/'private').mkdir(mode=0o700,exist_ok=True)
    logfile=RotatingFileHandler(ROOT/'logs/service.log',maxBytes=2_000_000,backupCount=2,encoding='utf-8')
    logging.basicConfig(level=logging.INFO,handlers=[logfile],format='%(asctime)s %(message)s')
    lock=open(ROOT/'private/runtime.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    kp=ROOT/'private/pairing.key'
    if not kp.exists():
        fd=os.open(kp,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as f:f.write(secrets.token_hex(32))
    key=kp.read_text().strip()
    rclpy.init();node=MapService(a.gateway_url,key);server=ThreadingHTTPServer((a.bind,a.port),handler(node,key));server.daemon_threads=True
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    print('G12_MAP_READY port=%d gateway=%s NAVIGATION_DISARMED'%(a.port,a.gateway_url),flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.mission.pause('SERVICE_STOP');node.bridge.close()
        node.trace.close()
        server.shutdown();server.server_close();node.stop_worker();node.stop_frontend();node.destroy_node();rclpy.shutdown();lock.close()

if __name__=='__main__':main()
