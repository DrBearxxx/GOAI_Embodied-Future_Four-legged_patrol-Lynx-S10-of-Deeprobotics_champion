"""Use confirmed map observations when independent local odometry is unavailable.

Both estimates are expressed in the frozen map frame. Local-odometry epoch
changes must not be exposed as a new navigation coordinate frame. A fallback
retains the actual map measurement timestamp and never refreshes stale data.
"""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation
from continuous_pose import map_frame


class PoseSources:
    def __init__(self):
        self.observations=deque(maxlen=8);self.key=None;self.generation=None
        self.map_estimate=None;self.source=None;self.switches=0
        self.last_result=None

    def update(self,anchor):
        if not anchor or anchor.get('confirmed') is not True:return
        stamp=anchor.get('measurement_mono');T=np.asarray(anchor.get('T'),dtype=float)
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or T.shape!=(4,4) or not np.isfinite(T).all():return
        generation=tuple(anchor['generation']);key=(generation,stamp)
        if key==self.key:return
        if self.generation is None or map_frame(generation)!=map_frame(self.generation):self.observations.clear()
        elif self.observations and stamp<=self.observations[-1][0]:return
        self.key=key;self.generation=generation;self.observations.append((stamp,T))
        recent=[(t,M) for t,M in self.observations if stamp-t<=.35]
        times=np.array([t-stamp for t,M in recent]);positions=np.array([M[:3,3] for t,M in recent])
        rotations=Rotation.from_matrix([T[:3,:3].T@M[:3,:3] for t,M in recent]).as_rotvec()
        values=np.column_stack((positions,rotations));filtered=T.copy();velocity=np.zeros(3);omega=None
        if len(recent)>=3 and np.ptp(times)>.08:
            # Causal linear fit evaluated at the newest measurement, preserving
            # constant-velocity motion rather than delaying it with an average.
            A=np.column_stack((np.ones(len(times)),times));base=np.exp(times/.18);weights=base.copy()
            for _ in range(3):
                fit=np.linalg.lstsq(A*np.sqrt(weights[:,None]),values*np.sqrt(weights[:,None]),rcond=None)[0]
                residual=np.linalg.norm((A@fit-values)[:,:3],axis=1)
                weights=base*np.minimum(1.,.035/np.maximum(residual,1e-9))
            filtered[:3,3]=fit[0,:3];filtered[:3,:3]=T[:3,:3]@Rotation.from_rotvec(fit[0,3:]).as_matrix()
            velocity=filtered[:3,:3].T@fit[1,:3]
            omega=fit[1,3:].tolist()
        self.map_estimate=dict(T=filtered.tolist(),pose=[*map(float,filtered[:3,3]),float(math.atan2(filtered[1,0],filtered[0,0]))],
            measurement_mono=stamp,estimate_mono=stamp,velocity_body=velocity.tolist(),
            angular_velocity_body=omega,
            quality_level=anchor.get('quality_level','tracking'),
            raw_map_pose=[*map(float,T[:3,3]),float(math.atan2(T[1,0],T[0,0]))])

    def choose(self,local,anchor,now):
        # A map innovation rejected by fusion must not re-enter through the
        # fallback path, especially during a short local scan gap.
        rejected=bool(local and anchor and anchor.get('measurement_mono')==local.get('rejected_map_measurement_mono'))
        if not rejected:self.update(anchor)
        valid_local=bool(local and local.get('mode') in ('TRACKING','DEGRADED','ODOM_BRIDGE','PREDICT_ONLY')
            and 0<=now-local.get('estimate_mono',-1e9)<=.25)
        fresh_map=bool(self.map_estimate and 0<=now-self.map_estimate['measurement_mono']<=.25)
        source=None
        if valid_local:
            result=dict(local);source='local_odometry'
        elif fresh_map:
            result=dict(self.map_estimate,mode='DEGRADED' if self.map_estimate['quality_level']=='degraded' else 'TRACKING',reason='',arrival_valid=True,
                map_age_s=now-self.map_estimate['measurement_mono'],odom_age_s=None,
                method='confirmed_map_causal_fit',budgets_calibrated=False,
                max_vx=None,max_wz=None,control_range_source='native_policy')
            source='confirmed_map_fallback'
        elif (self.map_estimate and self.map_estimate.get('angular_velocity_body') is not None
                and .25<now-self.map_estimate['measurement_mono']<=.45):
            # Bridge only the same short observation window used by LiDAR.
            # Velocity comes from distinct accepted map measurements, never
            # commanded motion, and arrival still needs an actual measurement.
            measured=self.map_estimate;dt=now-measured['measurement_mono'];T=np.array(measured['T'])
            v=np.array(measured['velocity_body']);omega=np.array(measured['angular_velocity_body'])
            T[:3,3]+=T[:3,:3]@v*dt
            T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(omega*dt).as_matrix()
            result=dict(measured,T=T.tolist(),pose=[*map(float,T[:3,3]),float(math.atan2(T[1,0],T[0,0]))],
                measurement_pose=list(measured['pose']),mode='PREDICT_ONLY',reason='',arrival_valid=False,estimate_mono=now,
                map_age_s=dt,odom_age_s=None,method='confirmed_map_short_prediction',budgets_calibrated=False,
                max_vx=None,max_wz=None,control_range_source='native_policy',
                prediction=dict(method='confirmed_map_velocity',horizon_s=dt))
            source='confirmed_map_fallback'
        else:
            # An unavailable source must not teleport the displayed robot back
            # to its older pose. Preserve the last delivered estimate, while
            # retaining its true age and explicitly reporting loss.
            if self.last_result is not None and self.last_result.get('generation')==['joint_map',*map_frame(self.generation or [])]:
                result=dict(self.last_result,mode='LOST',reason='LOCAL_ODOMETRY_STALE',arrival_valid=False,max_vx=0.,max_wz=0.)
            else:result=dict(local) if local else None
        if result is not None and self.generation is not None:
            result['odometry_generation']=result.get('odometry_generation',result.get('generation'))
            result['generation']=['joint_map',*map_frame(self.generation)]
            result['map_registration_generation']=list(self.generation)
            result['measurement_source']=source or 'unavailable'
            if source is not None:
                if self.source is not None and source!=self.source:self.switches+=1
                self.source=source
            result['pose_source_switches']=self.switches
            if source is not None:self.last_result=dict(result)
        return result
