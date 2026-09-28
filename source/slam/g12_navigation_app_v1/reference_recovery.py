"""Use an available map observation to replace a failed local reference."""
import numpy as np
from continuous_pose import map_frame

class ReferenceRecovery:
    def __init__(self):self.pending_epoch=None
    def observe(self,frontend,anchor,now,rejected_map_time=None):
        odom=frontend.get('odom') or {};stamp=odom.get('t')
        if (stamp is None or now-stamp<=.8 or frontend.get('reason')!='LOCAL_GEOMETRY_REJECTED'
                or self.pending_epoch==odom.get('epoch')):return None
        if not anchor or anchor.get('confirmed') is not True:return None
        t=anchor.get('measurement_mono');T=np.asarray(anchor.get('T'),dtype=float)
        if (not isinstance(t,(int,float)) or not 0<=now-t<=.45 or t==rejected_map_time
                or T.shape!=(4,4) or not np.isfinite(T).all()):return None
        self.pending_epoch=odom.get('epoch')
        return dict(reason='LOCAL_REFERENCE_STALE_WITH_USABLE_MAP',old_epoch=odom.get('epoch'),
            old_measurement_mono=stamp,anchor_times=[t],map_frame=list(map_frame(anchor.get('generation',[]))),
            local_age_s=now-stamp)
