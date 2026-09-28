"""Frozen indoor map localization. Publishes diagnostics, never native SDK commands."""
import argparse,copy,json,queue,sys,threading,time,cProfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT
from indoor.clock import use_live_clocks
from s10nav.util import read,sha
from s10nav.engine import Engine
from s10nav.matching import Matcher
from indoor.messages import convert
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from rclpy.clock import Clock,ClockType
from sensor_msgs.msg import Imu,PointCloud2,Image,CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class Localizer(Node):
    def __init__(self,a):
        super().__init__('wym_indoor_localizer');self.cfg=read(ROOT/'config.json');self.assets=ROOT/'assets'
        for f,h in read(self.assets/'manifest.json')['files'].items():
            if sha(self.assets/f)!=h:raise ValueError('asset_hash_mismatch:'+f)
        initial=None
        if a.initial:
            initial=np.eye(4);initial[:3,3]=a.initial[:3];initial[:3,:3]=Rotation.from_euler('z',np.radians(a.initial[3])).as_matrix()
        self.engine=use_live_clocks(Engine(self.assets,self.cfg,Matcher(self.assets,self.cfg),initial))
        self.ingest=use_live_clocks(Engine(self.assets,self.cfg,None));self.lock=threading.Lock();self.inbox=queue.Queue(maxsize=2)
        self.error=None;self.K=None;self.snapshot=None;self.seq=0;self.drops=0;self.stop=threading.Event();self.receipt_stats={};self.last_timings=None
        self.worker_profile=cProfile.Profile() if a.profile else None
        (ROOT/'live_logs').mkdir(exist_ok=True);self.run_id=time.time_ns();self.log=(ROOT/'live_logs'/f'localize-{self.run_id}.jsonl').open('x',buffering=1)
        self.health_log=(ROOT/'live_logs'/f'health-{self.run_id}.jsonl').open('x',buffering=1)
        self.publisher=self.create_publisher(String,'/wym/indoor/localization',1)
        topics=[(f'/wym/slam/{s}/imu',Imu) for s in ['front','rear']]+[(f'/wym/slam/{s}/points',PointCloud2) for s in ['front','rear']]
        topics += [('/wym/slam/insight9/imu',Imu),('/wym/slam/insight9/odometry',Odometry),('/wym/slam/insight9/depth/image_rect_raw',Image)]
        # Five queued samples cover only 25 ms at 200 Hz. Preserve IMU history
        # during brief GIL/registration stalls; source-age tests still discard
        # genuinely stale data, and no missing gyro is interpolated across.
        self.subs=[self.create_subscription(cls,t,lambda m,info,t=t:self.input(t,m,info),
            QoSProfile(depth=64 if cls==Imu else 20 if cls==Odometry else 2,reliability=ReliabilityPolicy.BEST_EFFORT)) for t,cls in topics]
        self.subs.append(self.create_subscription(CameraInfo,'/wym/slam/insight9/infra1/camera_info',self.camera_info,qos_profile_sensor_data))
        self.worker=threading.Thread(target=self.work,daemon=True);self.worker.start()
        self.timer=self.create_timer(.1,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('INDOOR LOCALIZATION ONLY: no SDK publisher; use fresh recording start hint only if physically confirmed.')

    def camera_info(self,m):
        K=np.array(m.k).reshape(3,3)
        if np.isfinite(K).all() and K[0,0]>0 and K[1,1]>0:self.K=K

    def input(self,topic,msg,info):
        now=time.monotonic();wall=time.time_ns();receipt_ns=info.get('received_timestamp',0)
        # DDS receipt precedes Python callback scheduling. Using callback entry
        # as receipt biases IMU clock offsets under load, making clouds appear
        # to come from the future. Reject unavailable/invalid middleware times
        # rather than silently declaring old buffered data fresh.
        lag=(wall-receipt_ns)*1e-9 if receipt_ns else float('inf')
        self.receipt_stats[topic]=dict(lag_s=lag if np.isfinite(lag) else None,received_timestamp_available=bool(receipt_ns))
        if not -.02<=lag<=.35:self.drops+=1;return
        received=now-lag
        if not topic.endswith('/points'):
            try:
                e=convert(topic,msg,received,self.K)
                if e:
                    with self.lock:self.ingest.event(e,now)
            except Exception as exc:self.error=repr(exc)
            return
        try:self.inbox.put_nowait((topic,msg,received))
        except queue.Full:
            try:self.inbox.get_nowait()
            except queue.Empty:pass
            self.drops+=1
            try:self.inbox.put_nowait((topic,msg,received))
            except queue.Full:self.drops+=1

    def work(self):
        if self.worker_profile:self.worker_profile.enable()
        while not self.stop.is_set():
            try:topic,msg,received=self.inbox.get(timeout=.05)
            except queue.Empty:continue
            try:
                now=time.monotonic()
                if now-received>.25:self.drops+=1;continue
                started=now;cpu_started=time.thread_time();event=convert(topic,msg,received,self.K);converted=time.monotonic()
                if event is None:continue
                with self.lock:
                    for s,c in self.ingest.clocks.items():
                        if c.epoch!=self.engine.clocks[s].epoch:self.engine.motion.gravity_anchor=None
                    self.engine.clocks=copy.deepcopy(self.ingest.clocks)
                    self.engine.motion.imu={s:q.copy() for s,q in self.ingest.motion.imu.items()}
                    self.engine.motion.vio=self.ingest.motion.vio.copy()
                    if 'depth' in self.ingest.perception:self.engine.perception['depth']=self.ingest.perception['depth']
                # Evaluate source age at actual processing time, after costly
                # PointCloud2 conversion and after taking the causal snapshot.
                now=time.monotonic();r=self.engine.event(event,now);after=time.monotonic();h=self.engine.poll(after);self.seq+=1
                mapped=self.engine.clocks[event['sensor']].map_stamp(event['stamp'],now)
                self.last_timings=dict(queue_ms=(started-received)*1000,convert_ms=(converted-started)*1000,
                    copy_ms=(now-converted)*1000,engine_ms=(after-now)*1000,thread_cpu_ms=(time.thread_time()-cpu_started)*1000,
                    measurement_age_ms=None if mapped is None else (after-mapped-float(np.max(event['rel'])))*1000,
                    raw_points=msg.width*msg.height,converted_points=len(event['points']))
                if r:r['pipeline']=self.last_timings
                if r and r.get('reason')=='cloud_source_age':
                    s=event['sensor'];mapped=self.engine.clocks[s].map_stamp(event['stamp'],now)
                    r['source_age_diagnostic']=dict(callback_queue_age_s=now-event['received'],scan_end_age_s=None if mapped is None else now-mapped-float(np.max(event['rel'])))
                if r:self.log.write(json.dumps(r)+'\n')
                if self.engine.T is None:self.snapshot=None;continue
                T=self.engine.T;fresh={s:False for s in ['front','rear']};obstacle=False
                for s,(t,p) in self.engine.perception.items():
                    if not 0<=after-t<.25:continue
                    if s in fresh:fresh[s]=True
                    # Preserve people for live collision stopping. This is NOT
                    # the human-filtered cloud used for offline map building.
                    rxy=np.linalg.norm(p[:,:2],axis=1)
                    hit=(rxy>.45)&(rxy<.85)&(p[:,2]>-.15)&(p[:,2]<1.2)
                    front=(p[:,0]>.45)&(p[:,0]<1.0)&(abs(p[:,1])<.45)&(p[:,2]>-.15)&(p[:,2]<1.2)
                    obstacle |= np.count_nonzero(hit|front)>=8
                self.snapshot=dict(seq=self.seq,mono=after,pose=[*map(float,T[:3,3]),float(np.arctan2(T[1,0],T[0,0]))],
                    health=h,generation=[self.engine.generation,*[self.engine.clocks[s].epoch for s in ['front','rear','camera']]],
                    front_fresh=fresh['front'],rear_fresh=fresh['rear'],obstacle=bool(obstacle),queue_drops=self.drops,error=self.error)
            except Exception as exc:self.error=repr(exc);self.snapshot=None;self.log.write(json.dumps(dict(error=self.error))+'\n')
        if self.worker_profile:
            self.worker_profile.disable();self.worker_profile.dump_stats(ROOT/'live_logs'/f'worker-{self.run_id}.prof')

    def publish(self):
        # mono deliberately remains the worker time: no fresh re-stamping of
        # an old pose. Independent SDK gate expires it even if this timer runs.
        if self.snapshot:
            m=String();m.data=json.dumps({**self.snapshot,'error':self.error});self.publisher.publish(m)
        now=time.monotonic();s=self.snapshot
        age=None if s is None else now-s['mono']
        valid=bool(s and age<.25 and s['health']['valid'] and s['health']['age_s']+age<.35 and not self.error)
        self.health_log.write(json.dumps(dict(mono=now,valid=valid,snapshot_age_s=age,snapshot=s,
            queue_drops=self.drops,error=self.error,ingest_counts=dict(self.ingest.counts),receipt_stats=self.receipt_stats,pipeline=self.last_timings))+'\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--initial',nargs=4,type=float);p.add_argument('--seconds',type=float,default=0);p.add_argument('--profile',action='store_true');a=p.parse_args()
    rclpy.init();n=Localizer(a);begin=time.monotonic()
    try:
        while rclpy.ok() and (not a.seconds or time.monotonic()-begin<a.seconds):rclpy.spin_once(n,timeout_sec=.03)
    except KeyboardInterrupt:pass
    finally:
        n.stop.set();n.worker.join(timeout=3)
        n.health_log.close()
        if not n.worker.is_alive():n.log.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
