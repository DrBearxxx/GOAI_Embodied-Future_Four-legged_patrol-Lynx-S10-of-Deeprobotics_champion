"""Shared AIRY wire-format and rigid-frame contract, independent of ROS runtime."""
import numpy as np

OUTPUT_DTYPE=np.dtype({'names':['x','y','z','intensity','time','ring'],
 'formats':['<f4']*5+['<u2'],'offsets':[0,4,8,12,16,20],'itemsize':24})

def normalize_points(msg):
    fields={f.name:f for f in msg.fields}
    required={'x':7,'y':7,'z':7,'intensity':7,'ring':4,'time':8}
    for name,kind in required.items():
        if name not in fields or fields[name].datatype!=kind or fields[name].count!=1:
            raise ValueError(f'AIRY field {name} must have datatype={kind}, count=1')
    order='>' if msg.is_bigendian else '<'
    dtype=np.dtype({'names':list(required),'formats':[order+{7:'f4',4:'u2',8:'f8'}[v] for v in required.values()],
                    'offsets':[fields[n].offset for n in required],'itemsize':msg.point_step})
    if msg.row_step<msg.point_step*msg.width or len(msg.data)<msg.row_step*msg.height:
        raise ValueError('Truncated PointCloud2 data')
    data=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
    good=np.isfinite(data['x']) & np.isfinite(data['y']) & np.isfinite(data['z']) & np.isfinite(data['intensity'])
    # A 10 Hz AIRY scan must carry finite relative seconds, not an absolute epoch.
    good &= np.isfinite(data['time']) & (data['time']>=0) & (data['time']<=1.0)
    valid=data[good]
    if len(valid)<2: raise ValueError('No usable timed points')
    result=np.zeros(len(valid),dtype=OUTPUT_DTYPE)
    for name in OUTPUT_DTYPE.names: result[name]=valid[name]
    # Both upstream parsers infer presence/end of timing from the final point.
    if np.any(np.diff(result['time'])<0): result=result[np.argsort(result['time'],kind='stable')]
    if result['time'][-1]<=0: raise ValueError('Point times have zero span')
    return result,{'input_points':len(data),'output_points':len(result),'invalid_points':int((~good).sum()),
                  'span_s':float(result['time'][-1]),'ring_max':int(result['ring'].max())}

def quaternion_matrix(q):
    x,y,z,w=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def matrix_quaternion(R):
    # Eigenvector formula remains stable at rotations close to pi.
    a=np.asarray(R);K=np.array([
      [a[0,0]-a[1,1]-a[2,2],a[0,1]+a[1,0],a[0,2]+a[2,0],a[2,1]-a[1,2]],
      [a[0,1]+a[1,0],a[1,1]-a[0,0]-a[2,2],a[1,2]+a[2,1],a[0,2]-a[2,0]],
      [a[0,2]+a[2,0],a[1,2]+a[2,1],a[2,2]-a[0,0]-a[1,1],a[1,0]-a[0,1]],
      [a[2,1]-a[1,2],a[0,2]-a[2,0],a[1,0]-a[0,1],np.trace(a)]])/3
    _,v=np.linalg.eigh(K);q=v[:,-1]
    return q if q[3]>=0 else -q

def transform(position,quaternion):
    T=np.eye(4);T[:3,:3]=quaternion_matrix(quaternion);T[:3,3]=position;return T

def body_pose(T_world_imu,T_body_imu):
    return T_world_imu@np.linalg.inv(T_body_imu)
