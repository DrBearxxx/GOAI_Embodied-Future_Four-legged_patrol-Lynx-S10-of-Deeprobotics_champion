"""Read-only ROS pointcloud matching probe. No publishers or action RPCs."""
from pathlib import Path
import json,sys,time
import numpy as np

stage=Path(__file__).resolve().parent
sys.path.insert(0,'/home/wym/s10_navigation_app_v1')
sys.path.insert(0,str(stage))
import paths
from automatic_localization import AutomaticMatcher,load_config
from seed_matcher import SeedMatcher,AppEngine
from registration_motion import use_registration_motion
from localization_quality import navigation_config
from native_nav.robust_timing import use_slewed_clocks
from indoor.pipeline import pack_cloud,snapshot_ingest
from s10nav.messages import convert
from registration_log import RegistrationLog
from continuous_pose import ContinuousPose
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import String

class Probe(Node):
    def __init__(self):
        super().__init__('goai_automatic_map_readonly_probe')
        self.cfg=navigation_config(json.loads((paths.BASE/'config.json').read_text()));self.cfg['threads']=2
        self.matcher=SeedMatcher(paths.ASSETS,self.cfg)
        self.engine=use_registration_motion(use_slewed_clocks(AppEngine(paths.ASSETS,self.cfg,self.matcher)))
        self.automatic=AutomaticMatcher(self.matcher,self.cfg,load_config(stage))
        self.fusion=ContinuousPose(load_config(stage));self.source_generation=None
        self.latest=None;self.state={};self.results=[]
        self.recorder=RegistrationLog(stage/'shadow-closed-loop-scans.bin',backups=1)
        qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.handles=[self.create_subscription(String,'/wym/g12/localization',self.status,10)]
        for side in ('front','rear'):
            for typ,name in ((Imu,'imu'),(PointCloud2,'points')):
                topic=f'/wym/slam/{side}/{name}'
                self.handles.append(self.create_subscription(typ,topic,lambda m,i,t=topic:self.input(t,m,i),qos if name=='imu' else QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)))
    def status(self,msg):
        self.state=json.loads(msg.data);solution=self.state.get('solution') or {};local=(self.state.get('local_odometry') or {}).get('odom')
        if not local or not solution.get('map_to_odom'):return
        if self.source_generation!=solution['generation']:
            self.fusion=ContinuousPose(load_config(stage));self.source_generation=solution['generation'];self.automatic.latest=None
        self.fusion.odometry(local)
        if self.fusion.offset is None:
            self.fusion.map_update(dict(T=(np.asarray(solution['map_to_odom'])@np.asarray(local['T'])).tolist(),
                measurement_mono=local['t'],generation=['shadow-initial',1],confirmed=True,odometry_epoch=local['epoch']))
    def input(self,topic,msg,info):
        now=time.monotonic();receipt=info.get('received_timestamp',0)
        age=(time.time_ns()-receipt)*1e-9 if receipt else 0
        if not -.02<=age<=.35:return
        received=now-age
        if topic.endswith('/imu'):
            event=convert(topic,msg,received,None)
            if event:self.engine.event(event,now)
        else:
            solution=self.state.get('solution') or {};local=self.state.get('local_odometry') or {};local=local.get('odom')
            if not local or self.fusion.offset is None:return
            item=pack_cloud(topic,msg,received,snapshot_ingest(self.engine,now))
            item.update(local_motion=local,map_alignment=dict(T=self.fusion.offset.tolist(),generation=[self.fusion.epoch,self.fusion.revision]))
            self.latest=item
    def match(self):
        now=time.monotonic();self.fusion.automatic_update(self.automatic.latest,now)
        estimate=self.fusion.estimate(now)
        item=self.latest;self.latest=None
        if item is None or time.monotonic()-item['received']>.25:return
        result=self.automatic.observe(item,self.engine,time.monotonic(),self.recorder)
        if result:
            result=dict(result,shadow_only=True,owner=self.state.get('navigation',{}).get('owner'),
                        lio_age_s=self.state.get('local_odometry',{}).get('age_s'),
                        correction_status=dict(self.fusion.auto_status),actual_pose=self.state.get('pose'),
                        shadow_pose=estimate['pose'] if estimate else None,
                        remaining_m=estimate['correction_remaining_m'] if estimate else None)
            self.results.append(result)
            print(json.dumps({k:result.get(k) for k in ('seq','sensor','accepted','reason','innovation_m','innovation_deg','quality','seconds')},ensure_ascii=False),flush=True)

rclpy.init();node=Probe();deadline=time.monotonic()+35
while rclpy.ok() and time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.005);node.match()
node.recorder.close()
(stage/'shadow-closed-loop-results.json').write_text(json.dumps(node.results,ensure_ascii=False,indent=2))
print(json.dumps(dict(samples=len(node.results),accepted=sum(r['accepted'] for r in node.results),recording=node.recorder.status())),flush=True)
node.destroy_node();rclpy.shutdown()
