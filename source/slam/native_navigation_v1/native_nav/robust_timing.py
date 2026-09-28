"""Causal clocks, bounded map corrections and measured odometry gap bridging."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from indoor.clock import LiveClock
from indoor.continuity import Continuity
from .odometry_bridge import measured_bridge,budgets
from .measured_motion import increment


class SlewedClock(LiveClock):
    # Slowly follow oscillator drift, not callback jitter. A sudden discrepancy
    # still goes through the original epoch/reset and queue-age checks.
    max_slew_per_second = .0002  # 200 ppm; <=10 us per nominal 50 ms update

    def __init__(self, config):
        super().__init__(config)
        self.last_adjust=None
        self.emitted=None
        self.emitted_epoch=0
        self.slew_total=0.

    def observe_imu(self,native,received):
        mapped=super().observe_imu(native,received)
        if self.epoch!=self.emitted_epoch:
            self.emitted=None
            self.last_adjust=None
            self.emitted_epoch=self.epoch
        if mapped is None:
            return None
        if self.last_adjust is None:
            self.last_adjust=received
        dt=received-self.last_adjust
        if dt>=.05:
            target=float(np.quantile(self.samples,.05))
            # Do not accumulate correction capacity while disconnected.
            budget=self.max_slew_per_second*min(dt,.1)
            change=max(-budget,min(budget,target-self.offset))
            self.offset+=change
            self.slew_total+=change
            self.last_adjust=received
            mapped=native+self.offset
        if self.emitted is not None and mapped<=self.emitted:
            self.reason='mapped_time_not_increasing'
            self.faults+=1
            return None
        self.emitted=mapped
        return mapped


def use_slewed_clocks(engine):
    engine.clocks={s:SlewedClock(engine.cfg['clock']) for s in ('front','rear','camera')}
    return engine


def causal_endpoints(now,motion,start):
    result=[]
    for sensor,q in motion.imu.items():
        # A future mapped head does not erase measured past history. Select an
        # actual received sample <= now, not now itself and not an extrapolation.
        for row in reversed(q):
            t=row[0]
            if t<=now:
                if start<=t and 0<=now-t<=.08:
                    result.append((t,sensor))
                break
    return sorted(set(result),reverse=True)


class RobustContinuity(Continuity):
    def __init__(self):
        super().__init__();self.input_key=None;self.rejected_map_innovations=0;self.bridge_cache=None

    def update(self,anchor,motion):
        if not anchor:return
        key=(tuple(anchor['generation']),anchor['measurement_mono'],anchor.get('confirmed'))
        if key==self.input_key:return
        self.input_key=key
        # A same-map unconfirmed candidate must not replace a still usable
        # confirmed anchor. Its age continues increasing; no freshness reset.
        if (anchor and self.anchor and self.anchor.get('confirmed') is True and
                anchor.get('confirmed') is not True and anchor['generation']==self.anchor['generation']):
            return
        candidate=dict(anchor);T=np.array(anchor['T'],float);t=anchor['measurement_mono']
        candidate['raw_T']=T.tolist();candidate['filter_residual_m']=0.;candidate['filter_residual_z_m']=0.
        candidate['filter_residual_yaw_rad']=0.;candidate['fusion_method']='raw_map_fit'
        # Registration noise is not body velocity. Prefer a real same-epoch
        # 200 ms VIO increment when it agrees with measured gyro.
        velocity_increment=increment(motion,t-.20,t)
        if velocity_increment is not None:
            candidate['velocity']=(velocity_increment[:3,:3].T@velocity_increment[:3,3]/.20).tolist()
            candidate['velocity_source']='measured_vio_gyro'
        else:candidate['velocity_source']='map_history_fallback'
        if (self.anchor and self.anchor.get('confirmed') is True and anchor.get('confirmed') is True and
                self.anchor['generation']==anchor['generation']):
            D=increment(motion,self.anchor['measurement_mono'],t)
            if D is not None:
                predicted=np.array(self.anchor['T'])@D
                delta=T[:3,3]-predicted[:3,3]
                rotation=Rotation.from_matrix(predicted[:3,:3].T@T[:3,:3]).as_rotvec()
                if np.linalg.norm(delta)>.24 or np.linalg.norm(rotation)>.15:
                    # Rejected geometry never refreshes measurement time. The
                    # previous anchor may only survive its existing budgets.
                    self.rejected_map_innovations+=1;return
                gain=.60;filtered=predicted.copy();filtered[:3,3]+=gain*delta
                filtered[:3,:3]=predicted[:3,:3]@Rotation.from_rotvec(gain*rotation).as_matrix()
                residual=(1-gain)*delta
                candidate.update(T=filtered.tolist(),fusion_method='measured_odom_bounded_map_correction',
                    filter_residual_m=float(np.linalg.norm(residual)),filter_residual_z_m=float(abs(residual[2])),
                    filter_residual_yaw_rad=float((1-gain)*np.linalg.norm(rotation)))
        if (self.anchor and self.last_key==(tuple(candidate['generation']),t) and
                candidate.get('confirmed') is True and self.anchor.get('confirmed') is not True):
            self.anchor=candidate
        else:super().update(candidate,motion)
        self.bridge_cache=None

    def estimate(self,now,motion):
        a=self.anchor
        if not a:
            return None
        t=a['measurement_mono'];age=now-t;T=np.array(a['T']);v=np.array(a['velocity'])
        pose=[*map(float,T[:3,3]),float(np.arctan2(T[1,0],T[0,0]))]
        raw=np.array(a.get('raw_T',a['T']));raw_pose=[*map(float,raw[:3,3]),float(np.arctan2(raw[1,0],raw[0,0]))]
        result=dict(mode='LOST',reason='MAP_OBSERVATION_EXPIRED',measurement_mono=t,estimate_mono=t,
            map_age_s=age,measured_pose=raw_pose,pose=pose,generation=[*a['generation'],self.reassociation],
            xy_budget_m=1.,yaw_budget_rad=1.,budgets_calibrated=False,method='none',max_vx=0.,max_wz=0.,
            velocity_body=v.tolist(),continuity_implementation='native_v3',recovery_revision='resilient_navigation_v1',
            fusion_method=a.get('fusion_method','raw_map_fit'),velocity_source=a.get('velocity_source','map_history_fallback'),
            filter_residual_m=a.get('filter_residual_m',0.),filter_residual_z_m=a.get('filter_residual_z_m',0.),
            filter_residual_yaw_rad=a.get('filter_residual_yaw_rad',0.),rejected_map_innovations=self.rejected_map_innovations)
        if a['confirmed'] and math.isfinite(age) and age>self.horizon:
            latest_vio=next(((row[0],row[2]) for row in reversed(motion.vio) if row[0]<=now),None)
            cache_key=(t,tuple(a['generation']),latest_vio)
            diagnostic={}
            if (self.bridge_cache and self.bridge_cache[0]==cache_key and age<=1.2 and
                    0<=now-self.bridge_cache[1]['estimate_mono']<=.06):
                # VIO is slower than publication. Reuse only its exact original
                # integration endpoint; age/budget keep increasing. Never
                # relabel cached IMU/VIO as a new measurement.
                bridge=dict(self.bridge_cache[1]);b=bridge['evidence']
                bridge['xy_budget_m'],bridge['yaw_budget_rad']=budgets(age,b['path_length_m'],b['turn_rad'])
            else:
                bridge=measured_bridge(now,a,motion,diagnostic)
                if bridge is not None:self.bridge_cache=(cache_key,bridge)
            if bridge is not None:
                predicted=T@bridge['D'];end=bridge['estimate_mono']
                return {**result,'mode':'ODOM_BRIDGE','reason':'MAP_FIT_GAP_MEASURED_ODOMETRY',
                    'pose':[*map(float,predicted[:3,3]),float(np.arctan2(predicted[1,0],predicted[0,0]))],
                    'estimate_mono':end,'imu_age_s':now-end,'xy_budget_m':bridge['xy_budget_m'],
                    'yaw_budget_rad':bridge['yaw_budget_rad'],'method':'measured_vio_and_contiguous_gyro',
                    'bridge':bridge['evidence'],'max_vx':.03,'max_wz':0. if now-end>.04 else .06}
            result['bridge_reject']=diagnostic.get('reason')
            if 'interval' in diagnostic:result['bridge_debug']=diagnostic
        if not a['confirmed'] or not math.isfinite(age) or age<0 or age>self.horizon:
            return result
        endpoints=causal_endpoints(now,motion,t)
        if not endpoints:
            return {**result,'reason':'NO_FRESH_PAST_IMU'}
        D=None;method='none';estimate_t=t
        for end,sensor in endpoints:
            D,method=motion.relative(t,end,v)
            if D is not None:
                estimate_t=end
                break
        if D is None:
            return {**result,'reason':'IMU_PATH_HAS_GAP_OR_NO_COVERAGE'}
        if not np.isfinite(v).all() or np.linalg.norm(v)>motion.cfg['max_speed_mps']:
            return {**result,'reason':'MOTION_OUTSIDE_PREDICTION_BUDGET'}
        predicted=T@D;travel=np.linalg.norm(D[:3,3]);xy=.04+.08*age+.15*age*age;yaw=.035+.10*age+.20*age*age
        if travel>.15 or xy>.14 or yaw>.18:
            return {**result,'reason':'PREDICTION_BUDGET_EXHAUSTED'}
        mode='TRACKING' if age<=.25 and len(a['healthy_lidars'])==2 else 'DEGRADED' if age<=.35 else 'PREDICT_ONLY'
        limits={'TRACKING':(.1,.2),'DEGRADED':(.04,.10),'PREDICT_ONLY':(.02,.06)}
        imu_age=now-estimate_t
        if imu_age>.04:
            mode='PREDICT_ONLY'
        return {**result,'mode':mode,'reason':'','pose':[*map(float,predicted[:3,3]),float(np.arctan2(predicted[1,0],predicted[0,0]))],
                'estimate_mono':estimate_t,'imu_age_s':imu_age,'xy_budget_m':xy,'yaw_budget_rad':yaw,
                'method':method,'endpoint_sensor':sensor,'max_vx':limits[mode][0],
                'max_wz':0. if imu_age>.04 else limits[mode][1]}
