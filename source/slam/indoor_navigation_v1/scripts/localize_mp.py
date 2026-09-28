"""Low-latency ROS ingress plus a separate non-actuating matching process."""
import argparse,json,multiprocessing as mp,queue,sys,time,fcntl
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT
from indoor.clock import use_live_clocks
from indoor.pipeline import worker_main,pack_cloud,snapshot_ingest,latest_put
from indoor.continuity import Continuity,use_redundancy,safety_policy
from indoor.safety_stream import safety_worker
from s10nav.util import read,sha
from s10nav.messages import convert
from s10nav.engine import Engine
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Imu,PointCloud2,Image,CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class Localizer(Node):
    def __init__(self,a):
        super().__init__('wym_indoor_localizer');self.cfg=read(ROOT/'config.json');assets=ROOT/'assets'
        for f,h in read(assets/'manifest.json')['files'].items():
            if sha(assets/f)!=h:raise ValueError('asset_hash_mismatch:'+f)
        self.asset_id=sha(assets/'manifest.json');self.boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.ingest=use_redundancy(use_live_clocks(Engine(assets,self.cfg,None)));self.K=None;self.error=None;self.snapshot=None
        self.continuity=Continuity();self.safety_records={};self.publish_seq=0;self.safety_drops=0
        self.drops=0;self.receipt_stats={};self.last_timings=None;self.run_id=time.time_ns();self.last_snapshot_t=-1.;self.ingest_snapshot=None
        self.input_rejects={}
        (ROOT/'live_logs').mkdir(exist_ok=True);self.log=(ROOT/'live_logs'/f'health-{self.run_id}.jsonl').open('x',buffering=1)
        ctx=mp.get_context('spawn');self.inboxes=[ctx.Queue(maxsize=1),ctx.Queue(maxsize=1)];self.outbox=ctx.Queue(maxsize=2);self.stop=ctx.Event()
        self.worker=ctx.Process(target=worker_main,args=(self.inboxes,self.outbox,self.stop,self.run_id,a.initial,a.profile),daemon=True)
        self.worker.start();self.pub=self.create_publisher(String,'/wym/indoor/localization',1)
        self.safety_boxes=[ctx.Queue(maxsize=1),ctx.Queue(maxsize=1)];self.safety_out=ctx.Queue(maxsize=2)
        self.safety_process=ctx.Process(target=safety_worker,args=(self.safety_boxes,self.safety_out,self.stop),daemon=True)
        self.safety_process.start()
        topics=[(f'/wym/slam/{s}/imu',Imu) for s in ('front','rear')]+[(f'/wym/slam/{s}/points',PointCloud2) for s in ('front','rear')]
        topics += [('/wym/slam/insight9/imu',Imu),('/wym/slam/insight9/odometry',Odometry),('/wym/slam/insight9/depth/image_rect_raw',Image)]
        self.subs=[self.create_subscription(cls,t,lambda m,info,t=t:self.input(t,m,info),QoSProfile(depth=64 if cls==Imu else 20 if cls==Odometry else 2,reliability=ReliabilityPolicy.BEST_EFFORT)) for t,cls in topics]
        self.subs.append(self.create_subscription(CameraInfo,'/wym/slam/insight9/infra1/camera_info',self.camera_info,qos_profile_sensor_data))
        self.timer=self.create_timer(.02,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
        print('LOCALIZATION ONLY, separate worker PID',self.worker.pid,'run',self.run_id,flush=True)

    def camera_info(self,m):
        K=np.array(m.k).reshape(3,3)
        if np.isfinite(K).all() and K[0,0]>0 and K[1,1]>0:self.K=K

    def input(self,topic,msg,info):
        now=time.monotonic();receipt=info.get('received_timestamp',0);lag=(time.time_ns()-receipt)*1e-9 if receipt else float('inf')
        self.receipt_stats[topic]=dict(lag_s=lag if np.isfinite(lag) else None,received_timestamp_available=bool(receipt))
        if not -.02<=lag<=.35:self.drops+=1;return
        received=now-lag
        try:
            if not topic.endswith('/points'):
                e=convert(topic,msg,received,self.K)
                if e:self.ingest.event(e,now)
                return
            # Copy only immutable, bounded numerical sensor history. Cloud
            # conversion and all map registration execute outside this GIL.
            data=snapshot_ingest(self.ingest,now)
            side=0 if '/front/' in topic else 1
            packed=pack_cloud(topic,msg,received,data)
            self.drops+=latest_put(self.inboxes[side],packed)
            sensor='front' if side==0 else 'rear';clock=self.ingest.clocks[sensor]
            native=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;mapped=clock.map_stamp(native,now)
            if mapped is not None:
                self.safety_drops+=latest_put(self.safety_boxes[side],dict(sensor=sensor,cloud=packed['cloud'],received=received,
                    native=native,epoch=clock.epoch,mapped_start=mapped))
        except (ValueError,TypeError,KeyError,IndexError,OverflowError) as exc:
            # A malformed packet is quarantined, not a permanent node fault.
            # It cannot refresh sensor evidence; ordinary expiry still applies.
            old=self.input_rejects.get(topic,{})
            self.input_rejects[topic]=dict(count=old.get('count',0)+1,last_mono=now,reason=repr(exc))

    def publish(self):
        while True:
            try:item=self.outbox.get_nowait()
            except queue.Empty:break
            if 'error' in item:self.error=item['error'];self.snapshot=None
            else:self.snapshot=item['snapshot'];self.last_timings=item['pipeline']
        now=time.monotonic()
        while True:
            try:item=self.safety_out.get_nowait();self.safety_records=item['records']
            except queue.Empty:break
        for sensor in list(self.safety_records):
            if self.safety_records[sensor]['epoch']!=self.ingest.clocks[sensor].epoch:del self.safety_records[sensor]
        if not self.worker.is_alive() and not self.stop.is_set():self.error='matching_worker_exited';self.snapshot=None
        if not self.safety_process.is_alive() and not self.stop.is_set():self.error='safety_worker_exited'
        s=self.snapshot;age=None if s is None else now-s['mono']
        valid=bool(s and age<.25 and s['health']['valid'] and s['health']['age_s']+age<.35 and not self.error)
        if s:
            self.continuity.update(s.get('anchor'),self.ingest.motion)
            solution=self.continuity.estimate(now,self.ingest.motion);safety=safety_policy(now,self.safety_records)
            self.publish_seq+=1
            message={**s,'error':self.error,'queue_drops':self.drops,
                'asset_id':self.asset_id,'boot_id':self.boot_id,'run_id':str(self.run_id),'matching_snapshot_mono':s['mono']}
            if solution:
                message.update(schema='goai.localization.continuity.v2',mono=now,seq=self.publish_seq,pose=solution['pose'],
                    generation=solution['generation'],solution=solution,safety=safety,safety_records=self.safety_records)
                message.pop('anchor',None)
            m=String();m.data=json.dumps(message);self.pub.publish(m)
        else:solution=None;safety=safety_policy(now,self.safety_records)
        self.log.write(json.dumps(dict(mono=now,valid=valid,snapshot_age_s=age,snapshot=s,error=self.error,queue_drops=self.drops,
            ingest_counts=dict(self.ingest.counts),receipt_stats=self.receipt_stats,pipeline=self.last_timings,
            solution=solution,safety=safety,safety_records=self.safety_records,safety_queue_drops=self.safety_drops,input_rejects=self.input_rejects))+'\n')

    def close(self):
        self.stop.set();self.worker.join(timeout=3)
        if self.worker.is_alive():self.worker.terminate();self.worker.join(timeout=2)
        self.safety_process.join(timeout=2)
        if self.safety_process.is_alive():self.safety_process.terminate();self.safety_process.join(timeout=2)
        for q in [*self.inboxes,self.outbox,*self.safety_boxes,self.safety_out]:q.cancel_join_thread();q.close()
        self.log.close();self.destroy_node()

def main():
    p=argparse.ArgumentParser();p.add_argument('--initial',nargs=4,type=float);p.add_argument('--seconds',type=float,default=0);p.add_argument('--profile',action='store_true');a=p.parse_args()
    lock=open('/tmp/wym-indoor-localizer.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init();n=Localizer(a);start=time.monotonic()
    try:
        while rclpy.ok() and (not a.seconds or time.monotonic()-start<a.seconds):rclpy.spin_once(n,timeout_sec=.01)
    except KeyboardInterrupt:pass
    finally:n.close();rclpy.shutdown()
if __name__=='__main__':main()
