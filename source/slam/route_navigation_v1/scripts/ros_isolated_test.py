"""Synthetic ROS transport/watchdog test. Refuses the robot's ROS domain.

No motor command types, services, SDK calls or TF broadcasters are used.
"""
import os,sys,time,json,argparse
from pathlib import Path
import numpy as np
if os.environ.get('ROS_DOMAIN_ID')!='189':raise SystemExit('Test requires isolated ROS_DOMAIN_ID=189, never robot domain 88')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,inv,apply,write
from ros_shadow import Shadow
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,PointCloud2,PointField
from std_msgs.msg import String

def stamp(header,t):
    ns=int(t*1e9);header.stamp.sec=ns//10**9;header.stamp.nanosec=ns%10**9

def main():
    rclpy.init();args=argparse.Namespace(assets=str(ROOT/'assets'),initial=[.5,.1,0,0],seconds=0,log_dir=str(ROOT/'live_logs/isolated_test'))
    shadow=Shadow(args);feed=Node('s10_isolated_test_source');executor=SingleThreadedExecutor();executor.add_node(shadow);executor.add_node(feed)
    records=[]
    feed.create_subscription(String,'/wym/route_nav/shadow_command',lambda msg:records.append((time.monotonic(),json.loads(msg.data))),10)
    ext=read(ROOT/'assets/extrinsics.json');points=np.load(ROOT/'assets/map.npy').astype(float);p=points-np.array([.5,.1,0]);p=p[(np.linalg.norm(p,axis=1)<9)&(np.linalg.norm(p,axis=1)>1)]
    if len(p)>8000:p=p[np.linspace(0,len(p)-1,8000,dtype=int)]
    pubs={};clouds={}
    for side in ('front','rear'):
        pubs[side,'imu']=feed.create_publisher(Imu,f'/wym/slam/{side}/imu',qos_profile_sensor_data)
        pubs[side,'cloud']=feed.create_publisher(PointCloud2,f'/wym/slam/{side}/points',qos_profile_sensor_data)
        raw=apply(inv(np.array(ext['lidar'][side])),p);a=np.zeros(len(raw),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('time','<f4')])
        for i,key in enumerate(['x','y','z']):a[key]=raw[:,i]
        a['time']=np.linspace(0,.1,len(raw));clouds[side]=a
    start=time.monotonic();last_imu=last_cloud=-1.
    try:
        while time.monotonic()-start<20:
            now=time.monotonic();t=now-start;wall=time.time();active=t<12 or t>=15
            if active and t-last_imu>=.01:
                last_imu=t
                for side in ('front','rear'):
                    m=Imu();m.header.frame_id=f'wym_{side}_imu';stamp(m.header,wall)
                    accel=np.array(ext['imu'][side])[:3,:3].T@np.array([0,0,9.81])
                    m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,accel)
                    pubs[side,'imu'].publish(m)
            if active and t>2 and t-last_cloud>=.2:
                last_cloud=t
                for side in ('front','rear'):
                    a=clouds[side];m=PointCloud2();m.header.frame_id=f'wym_{side}_lidar';stamp(m.header,wall-.11)
                    m.height=1;m.width=len(a);m.is_dense=True;m.point_step=16;m.row_step=16*len(a)
                    m.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z','time'])];m.data=a.tobytes()
                    pubs[side,'cloud'].publish(m)
            executor.spin_once(timeout_sec=.001)
    finally:
        shadow.stop_event.set();shadow.worker.join(timeout=10)
        if not shadow.worker.is_alive():shadow.log.close()
        shadow.destroy_node();feed.destroy_node();rclpy.shutdown()
    outage=[v for t,v in records if 13<=t-start<15];pre=[v for t,v in records if 5<=t-start<12]
    result=dict(domain=189,samples=len(records),valid_before_outage=sum(v['localization']['valid'] for v in pre),outage_samples=len(outage),
        valid_during_outage=sum(v['localization']['valid'] for v in outage),nonzero_during_outage=sum(bool(v['vx'] or v['wz']) for v in outage),
        motion_authorized_count=sum(v['motion_authorized'] for _,v in records),worker_error=shadow.error,queue_drops=shadow.queue_drops,counts=dict(shadow.engine.counts),
        test_scope='Synthetic static-map ROS transport and timeout test, NOT real-sensor localization accuracy',robot_commands_sent=False)
    result['passed']=bool(pre) and result['valid_before_outage']>0 and len(outage)>5 and result['valid_during_outage']==result['nonzero_during_outage']==result['motion_authorized_count']==0 and not shadow.error
    write(ROOT/'live_logs/isolated_ros_test.json',result);print(json.dumps(result),flush=True)
    if not result['passed']:raise SystemExit(1)
if __name__=='__main__':main()
