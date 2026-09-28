"""Optional received VIO + contiguous gyro, never fabricates camera samples."""
import numpy as np
from native_nav.measured_motion import increment
from native_nav.odometry_bridge import measured_bridge
from native_nav.robust_timing import causal_endpoints


def bridge_local(sample,now,motion):
    start=sample['t'];age=now-start
    if not .02<age<=1.2 or sample.get('accepted',0)<3:return None
    if age>.55:
        return measured_bridge(now,dict(measurement_mono=start,confirmed=True,
            velocity=sample.get('velocity',[0.,0.,0.])),motion)
    vio=next((v for v in reversed(motion.vio) if v[0]<=now),None)
    endpoints=causal_endpoints(now,motion,start)
    if vio is None or not endpoints:return None
    end=min(vio[0],endpoints[0][0])
    if not 0<=now-end<=.08:return None
    D=increment(motion,start,end)
    if D is None or np.linalg.norm(D[:3,3])>.15:return None
    return dict(D=D,estimate_mono=end,xy_budget_m=.04+.03*age,yaw_budget_rad=.04+.04*age,
        evidence=dict(extrapolated=False,start_mono=start,end_mono=end,
                      path_length_m=float(np.linalg.norm(D[:3,3])),source='received_vio_gyro'))
