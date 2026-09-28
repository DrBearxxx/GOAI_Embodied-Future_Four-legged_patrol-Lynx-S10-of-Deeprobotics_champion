"""Persistent attitude search prior, separate from accepted LiDAR odometry.

Continue integrating observed short IMU intervals while geometric registration
is unavailable. Never integrate a many-second old velocity or mark this prior
as measured displacement. Gravity is a soft tilt observation, not an IMU gate.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def align_vector(a,b):
    a=np.asarray(a)/np.linalg.norm(a);b=np.asarray(b)/np.linalg.norm(b)
    axis=np.cross(a,b);s=np.linalg.norm(axis);c=float(np.clip(a@b,-1.,1.))
    if s<1e-9:
        if c>0:return np.zeros(3)
        axis=np.cross(a,[1.,0.,0.] if abs(a[0])<.8 else [0.,1.,0.]);axis/=np.linalg.norm(axis)
        return axis*np.pi
    return axis/s*math.atan2(s,c)


def gravity_observation(motion,end):
    observations=[]
    for sensor,q in getattr(motion,'imu',{}).items():
        samples=[s for s in q if end-.35<=s[0]<=end+.005]
        if not samples:continue
        a=np.asarray([s[2] for s in samples]);g=np.asarray([s[1] for s in samples])
        valid=np.isfinite(a).all(axis=1)&np.isfinite(g).all(axis=1);a=a[valid];g=g[valid]
        if not len(a):continue
        v=np.median(a,axis=0);norm=np.linalg.norm(v)
        if norm<1e-6:continue
        spread=float(np.median(np.linalg.norm(a-v,axis=1)))
        rate=float(np.median(np.linalg.norm(g,axis=1)))
        age=max(0.,end-samples[-1][0])
        weight=math.exp(-age/.3)/(1+((norm-9.81)/1.5)**2+(spread/2.)**2+(rate/1.5)**2)
        observations.append(dict(sensor=sensor,unit=v/norm,weight=weight,age_s=age,norm=norm,spread=spread,rate=rate))
    if not observations:return None
    # The best current sensor avoids diluting a valid gravity vector with a
    # vibrating or time-shifted redundant sensor. All weights are diagnostic.
    return max(observations,key=lambda x:x['weight'])


class RegistrationPrior:
    def __init__(self):
        self.R=None;self.t=None;self.observed_t=None;self.up=None
        self.gravity=None;self.details={};self.dt=0.

    def predict(self,T,stamp,end,motion,velocity):
        if self.R is None or stamp!=self.observed_t:
            self.R=np.asarray(T[:3,:3]).copy();self.t=end if stamp is None else stamp;self.observed_t=stamp
        start=self.t;late=end<start;dt=max(0.,end-start);self.dt=dt
        D,method=motion.relative(min(start,end),max(start,end),np.zeros(3)) if end!=start else (np.eye(4),'same_time')
        rotation=self.R.copy()
        if D is not None:rotation=rotation@(D[:3,:3].T if late else D[:3,:3])
        else:method='last_lidar_pose_recovery_seed'
        if not late:self.t=end
        self.gravity=gravity_observation(motion,end)
        if self.gravity is not None and self.up is None:self.up=rotation@self.gravity['unit']
        age=0. if stamp is None else max(0.,end-stamp)
        result=np.array(T,copy=True);result[:3,:3]=rotation
        # Translation is only a short search seed; it never grows throughout a
        # loss of overlap or while the robot is being lifted.
        if age<=.6:result[:3,3]+=T[:3,:3]@np.asarray(velocity)*age
        if age>.6:result=self.correct_tilt(result,max(dt,.05))
        if not late:self.R=result[:3,:3].copy()
        self.details=dict(method='persistent_attitude_prior',increment_method=method,
            increment_s=dt,measurement_age_s=age,translation_propagated=age<=.6,
            gravity=None if self.gravity is None else {k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in self.gravity.items()})
        return result

    def correct_tilt(self,T,dt):
        if self.up is None or self.gravity is None:return T
        correction=align_vector(T[:3,:3]@self.gravity['unit'],self.up)
        # Smooth complementary tilt observation. It leaves rotation about
        # gravity (heading) free and follows actual body pitch on stairs.
        gain=-math.expm1(-max(0.,dt)*self.gravity['weight']/2.5)
        result=T.copy();result[:3,:3]=Rotation.from_rotvec(correction*gain).as_matrix()@T[:3,:3]
        return result
