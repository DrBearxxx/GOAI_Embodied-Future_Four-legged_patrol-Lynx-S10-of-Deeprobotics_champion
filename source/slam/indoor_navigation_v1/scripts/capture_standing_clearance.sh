#!/bin/bash
# Read-only raw LiDAR snapshot for explaining obstacle detections.
set -eo pipefail
source /home/wym/s10_slam/env.sh
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1
python3 - <<'PY'
import json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/wym/s10_indoor_navigation_v1')
from indoor.pipeline import pack_cloud
from indoor.safety_stream import raw_points,clearance
from indoor.bootstrap import ROOT
import rclpy
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
out=ROOT/'results/standing-retest-20260914-002715';assert out.is_dir()
ext=json.loads((ROOT/'assets/extrinsics.json').read_text());clouds={};records={}
rclpy.init();node=rclpy.create_node('wym_standing_clearance_snapshot',enable_rosout=False,start_parameter_services=False)
def cb(side,msg):
    if side in clouds:return
    now=time.monotonic();p=raw_points(pack_cloud('',msg,now,{})['cloud'])[:,:3]
    X=np.array(ext['lidar'][side]);p=p@X[:3,:3].T+X[:3,3];x,y,z=p.T;r=np.hypot(x,y)
    hit=(z>-.15)&(z<1.2)&(((r>.45)&(r<.95))|((x>.45)&(x<1.1)&(abs(y)<.55)))
    clouds[side]=p;clouds[side+'_hit']=p[hit]
    records[side]=dict(**clearance(p,now,0),points=len(p),frame=msg.header.frame_id,
        hit_xyz_quantiles=np.quantile(p[hit],[0,.1,.5,.9,1],axis=0).tolist() if hit.any() else [],
        hit_by_quadrant={name:int(np.count_nonzero(hit&mask)) for name,mask in
            [('front_left',(x>=0)&(y>=0)),('front_right',(x>=0)&(y<0)),('rear_left',(x<0)&(y>=0)),('rear_right',(x<0)&(y<0))]})
subs=[node.create_subscription(PointCloud2,f'/wym/slam/{s}/points',lambda m,s=s:cb(s,m),Q) for s in ('front','rear')]
start=time.monotonic()
while time.monotonic()-start<6 and len(records)<2:rclpy.spin_once(node,timeout_sec=.01)
np.savez_compressed(out/'clearance_snapshot.npz',**clouds)
(out/'clearance_snapshot.json').write_text(json.dumps(records,indent=2)+'\n');print(json.dumps(records,indent=2))
node.destroy_node();rclpy.shutdown()
PY
