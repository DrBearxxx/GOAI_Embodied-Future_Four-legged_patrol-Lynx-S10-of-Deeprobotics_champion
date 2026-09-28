"""Receive live ROS sensors, publish diagnostics only. No /cmd_vel, SDK or TF writes."""
import argparse,json,queue,sys,threading,time,copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,sha
from s10nav.matching import Matcher
from s10nav.engine import Engine
from s10nav.messages import convert
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.clock import Clock,ClockType
from sensor_msgs.msg import Imu,PointCloud2,Image,CameraInfo
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class Shadow(Node):
    def __init__(self,args):
        super().__init__('s10_route_shadow');self.args=args;self.cfg=read(ROOT/'config.json');self.assets=Path(args.assets)
        manifest=read(self.assets/'manifest.json')
        for name,expected in manifest['files'].items():
            if sha(self.assets/name)!=expected:raise ValueError('Asset checksum mismatch: '+name)
        matcher=Matcher(self.assets,self.cfg);initial=None
        if args.initial:
            initial=np.eye(4);initial[:3,3]=args.initial[:3];initial[:3,:3]=Rotation.from_euler('z',np.deg2rad(args.initial[3])).as_matrix()
        self.engine=Engine(self.assets,self.cfg,matcher,initial);self.inbox=queue.Queue(maxsize=2)
        # Fast native-clock/IMU/VIO ingestion must not queue behind GICP.
        # The matching worker gets a causal, immutable snapshot per cloud.
        self.ingest=Engine(self.assets,self.cfg,None);self.ingest_lock=threading.Lock()
        self.stop_event=threading.Event();self.snapshot=None;self.latest_result=None;self.queue_drops=0;self.error=None;self.K=None
        self.start=time.monotonic();self.last_publish_stamp=-1.;self.report_dir=Path(args.log_dir);self.report_dir.mkdir(parents=True,exist_ok=True)
        self.log=(self.report_dir/f'shadow-{time.time_ns()}.jsonl').open('x',buffering=1)
        prefix='/wym/route_nav/'
        self.health_pub=self.create_publisher(String,prefix+'health',10);self.proposal_pub=self.create_publisher(String,prefix+'shadow_command',10)
        self.pose_pub=self.create_publisher(PoseStamped,prefix+'diagnostic_pose',10)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.route_pub=self.create_publisher(RosPath,prefix+'route',latched)
        self.topics={}
        for side in ['front','rear']:
            self.subscribe(f'/wym/slam/{side}/imu',Imu)
            self.subscribe(f'/wym/slam/{side}/points',PointCloud2)
        self.subscribe('/wym/slam/insight9/imu',Imu);self.subscribe('/wym/slam/insight9/odometry',Odometry)
        self.subscribe('/wym/slam/insight9/depth/image_rect_raw',Image)
        self.create_subscription(CameraInfo,'/wym/slam/insight9/infra1/camera_info',self.camera_info,qos_profile_sensor_data)
        route=RosPath();route.header.frame_id='joint_map'
        for p in read(self.assets/'route.json')['waypoints']:
            pose=PoseStamped();pose.header.frame_id='joint_map';pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=p['xyz']
            q=Rotation.from_euler('z',p['yaw']).as_quat();pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q)
            route.poses.append(pose)
        self.route_pub.publish(route)
        self.worker=threading.Thread(target=self.work,daemon=True);self.worker.start()
        self.timer=self.create_timer(.1,self.publish_status,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('SHADOW ONLY: no command publisher, robot SDK or TF broadcaster; route positions/order preserved.')

    def camera_info(self,m):
        K=np.array(m.k).reshape(3,3)
        if np.isfinite(K).all() and K[0,0]>0 and K[1,1]>0:self.K=K

    def subscribe(self,topic,typ):
        def callback(m):
            now=time.monotonic();self.topics[topic]=now
            if not topic.endswith('/points'):
                try:
                    e=convert(topic,m,now,self.K)
                    if e is not None:
                        with self.ingest_lock:self.ingest.event(e,now)
                except Exception as exc:self.error=type(exc).__name__+': '+str(exc)
                return
            item=(topic,m,now,self.get_clock().now().nanoseconds)
            try:self.inbox.put_nowait(item)
            except queue.Full:
                try:self.inbox.get_nowait()
                except queue.Empty:pass
                self.queue_drops+=1
                try:self.inbox.put_nowait(item)
                except queue.Full:self.queue_drops+=1
        self.create_subscription(typ,topic,callback,qos_profile_sensor_data)

    def work(self):
        while not self.stop_event.is_set():
            try:topic,msg,received,ros_ns=self.inbox.get(timeout=.05)
            except queue.Empty:continue
            try:
                now=time.monotonic()
                if now-received>self.cfg['clock']['max_queue_age_s']:
                    self.queue_drops+=1;continue
                e=convert(topic,msg,received,self.K)
                if e is None:continue
                with self.ingest_lock:
                    for s,c in self.ingest.clocks.items():
                        if self.engine.clocks[s].epoch!=c.epoch:self.engine.motion.gravity_anchor=None
                    self.engine.clocks=copy.deepcopy(self.ingest.clocks)
                    # Entries are immutable after append; shallow deque copies
                    # avoid copying thousands of small IMU arrays every scan.
                    self.engine.motion.imu={s:q.copy() for s,q in self.ingest.motion.imu.items()}
                    self.engine.motion.vio=self.ingest.motion.vio.copy()
                    if 'depth' in self.ingest.perception:self.engine.perception['depth']=self.ingest.perception['depth']
                result=self.engine.event(e,now)
                after=time.monotonic();proposal=self.engine.proposal(after)
                # Capture independent publication time: a blocked worker cannot
                # keep its last valid result alive through the timer watchdog.
                self.snapshot=(after,proposal)
                if result is not None:
                    self.log.write(json.dumps(result)+'\n')
                    if result['accepted']:self.latest_result=(received,ros_ns,result)
            except Exception as exc:
                self.error=type(exc).__name__+': '+str(exc);self.snapshot=None
                self.log.write(json.dumps(dict(error=self.error,t=time.monotonic()))+'\n')

    def publish_status(self):
        now=time.monotonic();snap=self.snapshot
        if snap is None:
            proposal=dict(state='HOLD_NO_DATA',vx=0.,vy=0.,wz=0.,motion_authorized=False,localization=dict(valid=False,state='WAIT_INPUT'))
        else:
            updated,proposal=snap;proposal=json.loads(json.dumps(proposal))
            h=proposal['localization'];age=(h.get('age_s') or 0)+now-updated
            if now-updated>.25 or age>self.cfg['health']['valid_age_s'] or self.error:
                proposal.update(state='HOLD_WATCHDOG',vx=0.,vy=0.,wz=0.)
                h.update(valid=False,state='STALE_WORKER',age_s=age)
        proposal.update(mode='shadow_only',motion_authorized=False,queue_drops=self.queue_drops,error=self.error)
        m=String();m.data=json.dumps(proposal);self.proposal_pub.publish(m)
        h=String();h.data=json.dumps({**proposal['localization'],'shadow_only':True,'queue_drops':self.queue_drops,'input_ages':{t:now-v for t,v in self.topics.items()}});self.health_pub.publish(h)
        if self.latest_result and proposal['localization']['valid']:
            received,ros_ns,result=self.latest_result
            if now-received<self.cfg['health']['valid_age_s'] and result['t']>self.last_publish_stamp:
                self.last_publish_stamp=result['t'];T=np.array(result['pose']);p=PoseStamped();p.header.frame_id='joint_map'
                # Map the causal monotonic measurement time to the ROS receipt
                # clock using this callback's clock pair, not a fresh restamp.
                ns=int(ros_ns+(result['t']-received)*1e9);p.header.stamp.sec=ns//10**9;p.header.stamp.nanosec=ns%10**9
                p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,T[:3,3]);q=Rotation.from_matrix(T[:3,:3]).as_quat()
                p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q);self.pose_pub.publish(p)

def main():
    p=argparse.ArgumentParser();p.add_argument('--assets',default=str(ROOT/'assets'));p.add_argument('--initial',nargs=4,type=float)
    p.add_argument('--seconds',type=float,default=0);p.add_argument('--log-dir',default=str(ROOT/'live_logs'));a=p.parse_args()
    rclpy.init();n=Shadow(a)
    try:
        while rclpy.ok() and (not a.seconds or time.monotonic()-n.start<a.seconds):rclpy.spin_once(n,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        n.stop_event.set();n.worker.join(timeout=10)
        from s10nav.util import write
        write(n.report_dir/f'shutdown-{time.time_ns()}.json',dict(seconds=time.monotonic()-n.start,topics=list(n.topics),counts=dict(n.engine.counts),ingest_counts=dict(n.ingest.counts),queue_drops=n.queue_drops,error=n.error,robot_commands_sent=False))
        if not n.worker.is_alive():n.log.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
