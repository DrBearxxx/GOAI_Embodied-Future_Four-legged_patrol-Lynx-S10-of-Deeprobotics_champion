"""Causal short-horizon navigation estimate. Budgets are NOT calibrated covariance."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from . import bootstrap
from s10nav.motion import Motion


class RedundantMotion(Motion):
    def gyro_path(self,start,end,preferred=None):
        path=super().gyro_path(start,end,preferred)
        if path is not None or end<start or end-start>self.cfg['max_predict_s']:return path
        # Stitch only measured, bracketing LiDAR IMU intervals. Never bridge a
        # common gap and never use acceleration double-integration as odometry.
        streams={}
        for name in ('front','rear'):
            q=[x for x in self.imu[name] if start-.04<=x[0]<=end+.04]
            if len(q)>=2:streams[name]=(np.array([x[0] for x in q]),np.array([x[1] for x in q]))
        times=np.unique(np.r_[start,end,*[ts[(ts>start)&(ts<end)] for ts,_ in streams.values()]])
        times=times[np.r_[True,np.diff(times)>1e-8]]
        rotations=[np.eye(3)];used=[];max_gap=0.;previous=None
        for a,b in zip(times,times[1:]):
            candidates={}
            for name,(ts,g) in streams.items():
                i=int(np.searchsorted(ts,(a+b)/2,side='right')-1);j=i+1
                if i<0 or j>=len(ts) or a<ts[i]-1e-8 or b>ts[j]+1e-8:continue
                gap=float(np.max(np.diff(ts[i:j+1])))
                if gap>self.cfg['max_imu_gap_s']:continue
                rate=np.array([np.interp((a+b)/2,ts,g[:,k]) for k in range(3)])
                candidates[name]=(rate,gap)
            if not candidates:return None
            # Where two streams overlap they must agree in calibrated body axes.
            if len(candidates)==2 and np.linalg.norm(candidates['front'][0]-candidates['rear'][0])>.15:return None
            name=previous if previous in candidates else next(iter(candidates))
            rate,gap=candidates[name]
            if previous is not None and name!=previous:
                pts,pg=streams[previous];nts,ng=streams[name]
                lo=max(pts[0],nts[0],a-.025);hi=min(pts[-1],nts[-1],a+.025)
                if hi-lo<.005:return None
                t=(lo+hi)/2
                if np.linalg.norm([np.interp(t,pts,pg[:,k])-np.interp(t,nts,ng[:,k]) for k in range(3)])>.15:return None
            rotations.append(rotations[-1]@Rotation.from_rotvec(rate*(b-a)).as_matrix())
            previous=name;used.append(name);max_gap=max(max_gap,gap)
        if len(rotations)<2:return None
        return dict(sensor='stitched_front_rear',times=times,rotations=np.array(rotations),max_gap=max_gap)


def use_redundancy(engine):
    engine.motion=RedundantMotion(engine.cfg['motion'],{s:np.array(T) for s,T in engine.ext['imu'].items()})
    return engine


class Continuity:
    horizon=.55

    def __init__(self):
        self.anchor=None;self.reassociation=0;self.last_key=None

    def update(self,anchor,motion):
        if not anchor:return
        key=(tuple(anchor['generation']),anchor['measurement_mono'])
        if key==self.last_key:return
        if self.anchor and key[0]==tuple(self.anchor['generation']):
            dt=anchor['measurement_mono']-self.anchor['measurement_mono']
            if dt<=0:return
            if .25<dt<=self.horizon:
                D,_=motion.relative(self.anchor['measurement_mono'],anchor['measurement_mono'],self.anchor['velocity'])
                if D is not None:
                    predicted=np.array(self.anchor['T'])@D;new=np.array(anchor['T'])
                    angle=Rotation.from_matrix(predicted[:3,:3].T@new[:3,:3]).magnitude()
                    if np.linalg.norm(predicted[:3,3]-new[:3,3])>.12+.08*dt or angle>.15:self.reassociation+=1
        self.anchor=anchor;self.last_key=key

    def estimate(self,now,motion):
        a=self.anchor
        if not a:return None
        t=a['measurement_mono'];age=now-t;T=np.array(a['T']);v=np.array(a['velocity'])
        result=dict(mode='LOST',reason='MAP_OBSERVATION_EXPIRED',measurement_mono=t,estimate_mono=t,
            map_age_s=age,measured_pose=[*map(float,T[:3,3]),float(np.arctan2(T[1,0],T[0,0]))],
            pose=[*map(float,T[:3,3]),float(np.arctan2(T[1,0],T[0,0]))],
            generation=[*a['generation'],self.reassociation],xy_budget_m=1.,yaw_budget_rad=1.,
            budgets_calibrated=False,method='none',max_vx=0.,max_wz=0.,velocity_body=v.tolist())
        if not a['confirmed'] or age<0 or age>self.horizon:return result
        # A complete gyro path is required even when a recent map fit exists.
        # Leave at most 80 ms unpropagated (20 mm/s, no turning after 40 ms).
        # The estimate timestamp stays at the last integrated IMU; no samples
        # are invented inside a common IMU gap, including after recovery.
        endpoints=sorted({min(now,q[-1][0]) for q in motion.imu.values() if q and -1e-9<=now-q[-1][0]<=.08},reverse=True)
        D=None;method='none';estimate_t=t
        for end in endpoints:
            if end<t:continue
            D,method=motion.relative(t,end,v)
            if D is not None:estimate_t=end;break
        if D is None:return {**result,'reason':'NO_CONTIGUOUS_GYRO'}
        if not np.isfinite(v).all() or np.linalg.norm(v)>motion.cfg['max_speed_mps']:return {**result,'reason':'MOTION_OUTSIDE_PREDICTION_BUDGET'}
        predicted=T@D;travel=np.linalg.norm(D[:3,3]);xy=.04+.08*age+.15*age*age;yaw=.035+.10*age+.20*age*age
        if travel>.15 or xy>.14 or yaw>.18:return {**result,'reason':'PREDICTION_BUDGET_EXHAUSTED'}
        mode='TRACKING' if age<=.25 and len(a['healthy_lidars'])==2 else 'DEGRADED' if age<=.35 else 'PREDICT_ONLY'
        limits={'TRACKING':(.1,.2),'DEGRADED':(.04,.10),'PREDICT_ONLY':(.02,.06)}
        imu_age=now-estimate_t
        if imu_age>.04:mode='PREDICT_ONLY'
        return {**result,'mode':mode,'reason':'','pose':[*map(float,predicted[:3,3]),float(np.arctan2(predicted[1,0],predicted[0,0]))],
            'estimate_mono':estimate_t,'imu_age_s':imu_age,'xy_budget_m':xy,'yaw_budget_rad':yaw,'method':method,
            'max_vx':limits[mode][0],'max_wz':0. if imu_age>.04 else limits[mode][1]}


def safety_policy(now,records):
    """Short stale-clear grace uses inflated zones; never turns in that grace."""
    if not isinstance(records,dict) or any(s not in records for s in ('front','rear')):return dict(mode='STOP',reason='NO_SAFETY_COVERAGE')
    try:
        ages=[now-records[s]['mono'] for s in ('front','rear')]
        if any(records[s]['blocked'] is not False or records[s]['valid'] is not True for s in ('front','rear')):
            return dict(mode='STOP',reason='OBSTACLE_OR_INVALID_CLOUD')
    except (KeyError,TypeError,ValueError):return dict(mode='STOP',reason='INVALID_SAFETY_RECORD')
    if any(not math.isfinite(v) or v<0 or v>.4 for v in ages):return dict(mode='STOP',reason='SAFETY_COVERAGE_EXPIRED')
    if max(ages)<=.25:return dict(mode='FRESH',reason='',max_vx=.1,max_wz=.2)
    return dict(mode='CAUTIOUS',reason='SHORT_CLEARANCE_GRACE',max_vx=.02,max_wz=0.)
