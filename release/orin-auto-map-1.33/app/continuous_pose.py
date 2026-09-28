"""Time-aligned map->odom correction. Never re-label predictions as observations."""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def inverse(T):
    out=np.eye(4); out[:3,:3]=T[:3,:3].T; out[:3,3]=-out[:3,:3]@T[:3,3]
    return out


def mix(a,b,u):
    T=np.eye(4);T[:3,3]=(1-u)*a[:3,3]+u*b[:3,3]
    T[:3,:3]=Slerp([0,1],Rotation.from_matrix([a[:3,:3],b[:3,:3]]))([u]).as_matrix()[0]
    return T


def map_frame(generation):
    # The first component changes on operator relocalization. The second is
    # the map matcher's internal reacquisition counter, not a new world frame.
    return tuple(generation[:1])


class ContinuousPose:
    def __init__(self,auto_config=None):
        self.history=deque(maxlen=160);self.epoch=None;self.offset=None;self.target=None
        self.map_t=None;self.map_generation=None;self.map_key=None;self.tick_t=None
        self.map_path=0.;self.rejected=0;self.revision=0;self.pending_jump=False
        self.rejection_streak=0
        self.offset_observations=deque(maxlen=9)
        self.auto_config=auto_config or {'enabled':False}
        self.auto_candidates=deque(maxlen=4);self.auto_seen=None
        self.auto_status=dict(enabled=bool(self.auto_config.get('enabled')),accepted=0,rejected=0,reason='WAIT_ALIGNMENT')

    def odometry(self,sample):
        if not sample or sample.get('t') is None:return
        t=float(sample['t']);T=np.array(sample['T'],float)
        if not math.isfinite(t) or T.shape!=(4,4) or not np.isfinite(T).all():return
        if sample['epoch']!=self.epoch:
            self.history.clear();self.epoch=sample['epoch'];self.offset=self.target=None
            self.map_t=self.map_key=None;self.revision+=1
            self.offset_observations.clear()
            self.auto_candidates.clear();self.auto_seen=None;self.tick_t=None
        if self.history and t<=self.history[-1]['t']:return
        self.history.append({**sample,'T':T})

    def at(self,t):
        for a,b in zip(self.history,list(self.history)[1:]):
            if a['t']<=t<=b['t'] and b['t']-a['t']<=.35:
                u=(t-a['t'])/(b['t']-a['t']);return mix(a['T'],b['T'],u), (1-u)*a['path']+u*b['path']
        for s in self.history:
            if abs(s['t']-t)<=.002:return s['T'],s['path']
        return None

    def map_update(self,anchor):
        """One time alignment per operator relocalization; never track map fits."""
        if not anchor or anchor.get('confirmed') is not True:return False
        if self.history and self.history[-1].get('source')=='lightning' and anchor.get('odometry_epoch')!=self.epoch:return False
        t=anchor['measurement_mono'];gen=tuple(anchor['generation']);key=(gen,t)
        if key==self.map_key:return False
        if self.map_generation is not None and map_frame(gen)==map_frame(self.map_generation):return False
        located=self.at(t)
        if located is None:return False
        T=np.array(anchor['T']);odom,path=located;target=T@inverse(odom)
        if not np.isfinite(target).all():return False
        self.offset=target.copy();self.target=target.copy();self.revision+=1
        self.map_t=t;self.map_generation=gen;self.map_key=key;self.map_path=path
        self.pending_jump=False;self.rejection_streak=0
        self.auto_candidates.clear();self.auto_seen=None;self.tick_t=None
        self.auto_status['reason']='WAIT_MAP_OBSERVATION'
        return True

    def automatic_update(self,observation,now):
        """Time-align independent local fits, then update a smooth correction target."""
        if not self.auto_config.get('enabled') or self.offset is None or not observation:return False
        stamp=observation.get('measurement_mono');key=(observation.get('seq'),stamp)
        if key==self.auto_seen:return False
        if observation.get('alignment_generation')!=[self.epoch,self.revision]:return False
        if stamp is None or not 0<=now-stamp<=self.auto_config['max_observation_age_s']:return False
        if self.map_t is not None and stamp<=self.map_t:return False
        located=self.at(stamp)
        if located is None:return False  # A delayed fit can wait for its true odometry bracket.
        self.auto_seen=key
        self.auto_status.update(reason=observation.get('reason',''),quality=observation.get('quality'),measurement_mono=stamp)
        if not observation.get('accepted'):
            self.auto_status['rejected']+=1;return False
        odom,path=located;T=np.asarray(observation['T'],float)
        current=self.offset@odom
        distance=float(np.linalg.norm(T[:3,3]-current[:3,3]))
        angle=math.degrees(Rotation.from_matrix(current[:3,:3].T@T[:3,:3]).magnitude())
        if distance>self.auto_config['max_innovation_m'] or angle>self.auto_config['max_innovation_deg']:
            self.auto_status.update(reason='OUTSIDE_LOCAL_CORRECTION',rejected=self.auto_status['rejected']+1);return False
        target=T@inverse(odom)
        self.auto_candidates=deque([(t,a) for t,a in self.auto_candidates if 0<stamp-t<=self.auto_config['consistency_window_s']],maxlen=4)
        consistent=[]
        for t,a in self.auto_candidates:
            previous=a@odom
            gap=float(np.linalg.norm(previous[:3,3]-T[:3,3]))
            turn=math.degrees(Rotation.from_matrix(previous[:3,:3].T@T[:3,:3]).magnitude())
            if gap<=self.auto_config['consistency_m'] and turn<=self.auto_config['consistency_deg']:consistent.append(a)
        self.auto_candidates.append((stamp,target))
        if not consistent:
            self.auto_status['reason']='ACCUMULATING_LOCAL_GEOMETRY';return False
        # Average agreeing observations at the current body, not around the map origin.
        mean=T.copy()
        for i,a in enumerate(consistent):mean=mix(mean,a@odom,1./(i+2))
        self.target=mean@inverse(odom);self.map_t=stamp;self.map_path=path
        self.auto_status.update(reason='SMOOTH_MAP_CORRECTION',accepted=self.auto_status['accepted']+1,
                                innovation_m=distance,innovation_deg=angle)
        return True

    def predict_lidar(self,now):
        """Brief constant-twist extrapolation of accepted LiDAR measurements.

        This bridges processing latency/scan gaps; it never renews the actual
        measurement time. No commanded velocity or fabricated IMU is used.
        """
        last=self.history[-1];dt=now-last['t']
        if not .25<dt<=.45 or last.get('accepted',0)<3 or 'velocity' not in last:return None
        v=np.asarray(last['velocity'],float)
        if v.shape!=(3,) or not np.isfinite(v).all():return None
        rates=[]
        for previous in self.history:
            span=last['t']-previous['t']
            if .15<=span<=.6 and previous.get('accepted',0)>=3:
                rates.append(Rotation.from_matrix(previous['T'][:3,:3].T@last['T'][:3,:3]).as_rotvec()/span)
        if not rates:return None
        omega=np.median(rates,axis=0);theta=float(np.linalg.norm(omega)*dt)
        w=omega*dt;W=np.array([[0.,-w[2],w[1]],[w[2],0.,-w[0]],[-w[1],w[0],0.]])
        J=np.eye(3)+(.5 if theta<1e-6 else (1-math.cos(theta))/theta**2)*W
        J+=(1/6 if theta<1e-6 else (theta-math.sin(theta))/theta**3)*(W@W)
        D=np.eye(4);D[:3,:3]=Rotation.from_rotvec(w).as_matrix();D[:3,3]=J@v*dt
        return D,dict(method='accepted_lidar_constant_twist',horizon_s=dt,
            velocity_body=v.tolist(),angular_velocity_body=omega.tolist())

    def estimate(self,now,motion=None):
        if self.offset is None or not self.history:return None
        last=self.history[-1];age=now-last['t'];map_age=now-self.map_t
        dist=angle=0.
        if self.auto_config.get('enabled') and self.target is not None:
            dt=0. if self.tick_t is None else min(.25,max(0.,now-self.tick_t))
            self.tick_t=now
            current=self.offset@last['T'];desired=self.target@last['T']
            corrected=mix(current,desired,-math.expm1(-dt/self.auto_config['smoothing_tau_s']))
            self.offset=corrected@inverse(last['T'])
            dist=float(np.linalg.norm(desired[:3,3]-corrected[:3,3]))
            angle=float(Rotation.from_matrix(corrected[:3,:3].T@desired[:3,:3]).magnitude())
        local_T=last['T'];estimate_t=last['t'];bridge=None;prediction=None
        if motion is not None and age>.18:
            from vio_continuity import bridge_local
            bridge=bridge_local(last,now,motion)
            if bridge is not None:
                local_T=local_T@bridge['D'];estimate_t=bridge['estimate_mono']
        lidar_prediction=self.predict_lidar(now) if bridge is None else None
        if lidar_prediction is not None:
            D,prediction=lidar_prediction;local_T=local_T@D;estimate_t=now
        if motion is not None and bridge is None and prediction is None and .25<age<=.45:
            from native_nav.robust_timing import causal_endpoints
            v=np.asarray(last.get('velocity',[0.,0.,0.]))
            if np.isfinite(v).all() and np.linalg.norm(v)<=.3:
                for end,sensor in causal_endpoints(now,motion,last['t']):
                    D,method=motion.relative(last['t'],end,v)
                    if D is not None and np.linalg.norm(D[:3,3])<=.08:
                        local_T=local_T@D;estimate_t=end
                        prediction=dict(method=method,imu_sensor=sensor,horizon_s=end-last['t'])
                        break
        T=self.offset@local_T;travel=max(0.,last['path']-self.map_path)
        if bridge:travel+=bridge['evidence']['path_length_m']
        budget=.04+.02*max(0.,map_age)+.025*travel+dist
        if prediction:budget+=.04*age+.15*age*age
        measured=0<=now-estimate_t<=.25 and last.get('accepted',0)>=1
        # Once aligned, fresh measured odometry remains usable through map
        # dropouts and rejected corrections. Global drift is corrected by an
        # operator relocalization; map age/travel are diagnostics, not stops.
        usable=measured and map_age>=0
        mode=('TRACKING' if map_age<=.45 and not bridge else 'ODOM_BRIDGE') if usable else 'LOST'
        quality_level=(last.get('quality') or {}).get('level','tracking')
        if usable and quality_level=='degraded':mode='DEGRADED'
        if usable and prediction:mode='PREDICT_ONLY'
        measured_T=self.offset@last['T']
        return dict(mode=mode,reason='' if usable else 'LOCAL_ODOMETRY_STALE',
            pose=[*map(float,T[:3,3]),float(np.arctan2(T[1,0],T[0,0]))],T=T.tolist(),
            measurement_pose=[*map(float,measured_T[:3,3]),float(np.arctan2(measured_T[1,0],measured_T[0,0]))],
            odom_T=last['T'].tolist(),map_to_odom=self.offset.tolist(),
            estimate_mono=estimate_t,measurement_mono=last['t'],map_measurement_mono=self.map_t,map_age_s=map_age,odom_age_s=age,
            generation=[self.epoch,self.revision],xy_budget_m=budget,yaw_budget_rad=.04+.03*max(0.,map_age)+angle,
            budgets_calibrated=False,method='local_odometry_with_smooth_map_correction' if self.auto_config.get('enabled') else 'local_odometry_with_operator_alignment',measurement_source=last.get('source','local_odometry'),map_association_enabled=bool(self.auto_config.get('enabled')),
            automatic_localization=dict(self.auto_status),
            correction_remaining_m=dist,correction_remaining_rad=angle,rejected_map_innovations=self.rejected,
            rejected_map_measurement_mono=self.map_key[1] if self.map_key and self.map_key[1]>self.map_t else None,
            max_vx=None if usable else 0.,max_wz=None if usable else 0.,control_range_source='native_policy',
            arrival_valid=usable and not bridge and not prediction,
            quality_level=quality_level,
            lidar_continuity=last.get('continuity'),
            prediction=prediction,
            vio_bridge=None if bridge is None else bridge['evidence'],
            velocity_body=last.get('velocity',[0.,0.,0.]),continuity_implementation='fixed_alignment_continuous_lidar_v5')
