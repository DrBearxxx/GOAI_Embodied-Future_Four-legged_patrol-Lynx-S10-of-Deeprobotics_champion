"""ROS data conversion shared by MCAP extraction and the live shadow adapter."""
import numpy as np
from .util import voxel,cap

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

def cloud(m,limit=5000):
    fields={f.name:f for f in m.fields};order='>' if m.is_bigendian else '<';fmt={7:'f4',8:'f8'}
    if not all(k in fields for k in ['x','y','z','time']):raise ValueError('XYZ and per-point time required')
    if len(m.data)!=m.row_step*m.height:raise ValueError('Invalid PointCloud2 buffer length')
    names=['x','y','z','time'];arrays=[]
    for k in names:
        f=fields[k]
        if f.datatype not in fmt:raise ValueError('Unsupported field datatype')
        arrays.append(np.ndarray((m.height,m.width),dtype=order+fmt[f.datatype],buffer=m.data,offset=f.offset,strides=(m.row_step,m.point_step)).ravel())
    p=np.column_stack(arrays);p=p[np.isfinite(p).all(1)]
    if not len(p):return np.empty((0,3)),np.empty(0)
    # Keep the actual scan time bounds even after deterministic spatial sampling.
    start,end=p[np.argmin(p[:,3])].copy(),p[np.argmax(p[:,3])].copy()
    p=cap(voxel(p,.12),limit-2);p=np.vstack([start,p,end])
    return p[:,:3].astype(np.float32),p[:,3].astype(np.float32)

def convert(topic,m,received,K=None):
    e=dict(received=float(received),stamp=stamp(m),frame=m.header.frame_id)
    if topic.endswith('/imu'):
        sensor='camera' if 'insight9' in topic else topic.split('/')[3]
        g=m.angular_velocity;a=m.linear_acceleration
        return {**e,'kind':'imu','sensor':sensor,'gyro':[g.x,g.y,g.z],'accel':[a.x,a.y,a.z]}
    if topic.endswith('/points'):
        p,t=cloud(m);return {**e,'kind':'cloud','sensor':topic.split('/')[3],'points':p,'rel':t}
    if topic.endswith('/odometry'):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        return {**e,'kind':'vio','sensor':'camera','child_frame':m.child_frame_id,'pose':[p.x,p.y,p.z,q.x,q.y,q.z,q.w]}
    if '/depth/' in topic and K is not None:
        if m.encoding not in ['16UC1','mono16']:raise ValueError('Expected millimetre uint16 depth')
        z=np.ndarray((m.height,m.width),dtype='>u2' if m.is_bigendian else '<u2',buffer=m.data,strides=(m.step,2)).astype(float)*.001
        v,u=np.mgrid[0:m.height:12,0:m.width:12];zz=z[v,u];ok=(zz>.4)&(zz<6)
        p=np.c_[(u[ok]-K[0,2])*zz[ok]/K[0,0],(v[ok]-K[1,2])*zz[ok]/K[1,1],zz[ok]]
        return {**e,'kind':'depth','sensor':'camera','points':p.astype(np.float32)}
    return None
