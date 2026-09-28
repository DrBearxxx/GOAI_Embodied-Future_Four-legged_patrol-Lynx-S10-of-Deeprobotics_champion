"""IMU-tolerant scan registration; estimates here are never pose observations.

Only the map/scan registration workers use this class. The published pose
continuity path keeps its measured-IMU rules. Missing gyro intervals therefore
produce registration hypotheses, not synthetic IMU samples or accepted poses.
"""
from collections import deque,Counter
import numpy as np
from scipy.spatial.transform import Rotation
import paths  # Register the existing frozen navigation dependency directories.
from indoor.continuity import RedundantMotion


class RegistrationMotion(RedundantMotion):
    def __init__(self,*args):
        super().__init__(*args)
        self.lidar_history=deque(maxlen=12);self.angular_velocity=None
        self.twist_time=None;self.deskew_counts=Counter();self.last_deskew=None

    def reset_lidar(self):
        """Discard old-frame motion estimates after operator relocalization."""
        self.lidar_history.clear();self.angular_velocity=None;self.twist_time=None

    def observe_lidar(self,t,T):
        """Called only after geometry accepts a real LiDAR observation."""
        T=np.asarray(T,float)
        if self.lidar_history and t<=self.lidar_history[-1][0]:return
        previous=[v for v in self.lidar_history if .15<=t-v[0]<=.6]
        if previous:
            stamp,old=previous[-1];rotation=Rotation.from_matrix(old[:3,:3].T@T[:3,:3]).as_rotvec()
            self.angular_velocity=T[:3,:3].T@old[:3,:3]@rotation/(t-stamp)
            self.twist_time=t
        self.lidar_history.append((t,T.copy()))

    def angular_estimate(self,start,end):
        # Use available recent gyro without claiming that it brackets the scan.
        for sensor in ('front','rear','camera'):
            samples=[v for v in self.imu[sensor] if start-.10<=v[0]<=end]
            if len(samples)>=2 and end-samples[-1][0]<=.15:
                return np.median([v[1] for v in samples],axis=0),'partial_imu:'+sensor
        if self.twist_time is not None and -.002<=end-self.twist_time<=.65:
            return self.angular_velocity.copy(),'accepted_lidar_twist'
        return np.zeros(3),'rigid_scan_initial_guess'

    def relative(self,start,end,velocity_body=None):
        D,method=super().relative(start,end,velocity_body)
        if D is not None:return D,method
        if not 0<=end-start<=self.cfg['max_predict_s']:return None,method
        omega,source=self.angular_estimate(start,end)
        v=np.zeros(3) if velocity_body is None else np.asarray(velocity_body,float)
        if not np.isfinite(v).all() or not np.isfinite(omega).all():return None,'invalid_registration_prediction'
        D=np.eye(4);D[:3,:3]=Rotation.from_rotvec(omega*(end-start)).as_matrix();D[:3,3]=v*(end-start)
        return D,'registration_prediction:'+source

    def deskew(self,points,rel,start,X,velocity_body):
        body,detail=super().deskew(points,rel,start,X,velocity_body)
        if body is not None:
            detail=dict(detail,method='measured_gyro');self.deskew_counts['measured_gyro']+=1
        else:
            rel=np.asarray(rel,float);end=start+float(np.max(rel))
            omega,source=self.angular_estimate(start,end)
            v=np.asarray(velocity_body,float)
            if not np.isfinite(omega).all() or not np.isfinite(v).all():return None,'nonfinite_motion_estimate'
            p=np.asarray(points)@X[:3,:3].T+X[:3,3];body=np.empty_like(p)
            bins=np.round(rel/.005)*.005
            for b in np.unique(bins):
                delta=float(np.clip(b,0.,rel.max())-rel.max());mask=bins==b
                R=Rotation.from_rotvec(omega*delta).as_matrix()
                body[mask]=p[mask]@R.T+v*delta
            detail=dict(method='estimated_motion_deskew',angular_source=source,missing_imu_reason=detail,
                        angular_velocity_radps=omega.tolist(),scan_duration_s=end-start,
                        requires_geometric_registration=True)
            self.deskew_counts[source]+=1
        self.last_deskew=detail
        return body,detail


def use_registration_motion(engine):
    engine.motion=RegistrationMotion(engine.cfg['motion'],{s:np.array(T) for s,T in engine.ext['imu'].items()})
    return engine
