"""Opportunistic local scan-to-map observations; never changes control ownership."""
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from localization_quality import classify


def load_config(root):
    path=Path(root)/'auto_localization.json'
    return json.loads(path.read_text()) if path.exists() else {'enabled':False}


class AutomaticMatcher:
    def __init__(self,matcher,cfg,options):
        self.matcher=matcher;self.cfg=cfg;self.options=options
        self.last_attempt=-math.inf;self.last_sensor=None;self.seq=0
        self.latest=None

    def observe(self,item,engine,now,recorder=None):
        """Use the continuous pose as a local seed, without global place retrieval."""
        alignment=item.get('map_alignment');local=item.get('local_motion')
        if not self.options.get('enabled') or not alignment or not local:return None
        if now-self.last_attempt<self.options['period_s']:return None
        sensor='front' if '/front/' in item['topic'] else 'rear'
        # Alternate when both sensors are available, without waiting on a missing one.
        if sensor==self.last_sensor and now-self.last_attempt<self.options['period_s']+.2:return None
        self.last_attempt=now;self.last_sensor=sensor;self.seq+=1
        result=dict(seq=self.seq,sensor=sensor,accepted=False,reason='NO_USABLE_GEOMETRY',
                    alignment_generation=alignment['generation'],odometry_epoch=local['epoch'])
        try:
            from indoor.pipeline import unpack_cloud
            from indoor.messages import convert
            from s10nav.util import voxel,cap
            e=convert(item['topic'],unpack_cloud(item['cloud']),item['received'])
            start=engine.clocks[sensor].map_stamp(e['stamp'],now)
            rel=np.asarray(e['rel']);points=np.asarray(e['points'])
            if start is None or len(rel)!=len(points) or not len(rel) or not np.isfinite(rel).all():raise ValueError('INVALID_SCAN_TIME')
            if rel.min()<-.001 or not .03<float(rel.max())<.16:raise ValueError('INVALID_SCAN_DURATION')
            end=start+float(rel.max());result['measurement_mono']=end
            if not -.025<=now-end<=.35:raise ValueError('OLD_SCAN')
            if alignment['generation'][0]!=local['epoch']:raise ValueError('ODOMETRY_EPOCH_CHANGED')
            dt=end-local['t']
            if not -.15<=dt<=.35:raise ValueError('ODOMETRY_SEED_UNAVAILABLE')
            velocity=np.asarray(local.get('velocity',[0.,0.,0.]));omega=np.asarray(local.get('angular_velocity',[0.,0.,0.]))
            if not np.isfinite(velocity).all() or not np.isfinite(omega).all():raise ValueError('INVALID_MOTION_SEED')
            D=np.eye(4);D[:3,:3]=Rotation.from_rotvec(omega*dt).as_matrix();D[:3,3]=velocity*dt
            guess=np.asarray(alignment['T'])@np.asarray(local['T'])@D
            body,deskew=engine.motion.deskew(points,rel,start,np.asarray(engine.ext['lidar'][sensor]),velocity)
            if body is None:raise ValueError('DESKEW_UNAVAILABLE')
            body=cap(voxel(body,self.cfg['scan_voxel_m']),self.cfg['scan_cap'])
            radius=np.linalg.norm(body,axis=1);body=body[(radius>.65)&(radius<32)]
            # The scoped manual matcher is idle here. No waypoint is used as a pose observation.
            fit=self.matcher.register(body,guess,wide=False)
            result.update(quality=fit.get('quality'),seconds=fit.get('seconds'),deskew=deskew,
                          initial_T=guess.tolist(),reasons=fit.get('reasons',[]))
            q=fit.get('quality') or {}
            level=classify(q.get('overlap'),q.get('median_nn_m'),q.get('condition'),
                           'not_converged' not in fit.get('quality_warnings',fit.get('reasons',[])))
            result['quality_level']=level
            # Poor map structure simply yields no correction. It never invalidates LIO.
            if fit.get('accepted') and level=='tracking':
                T=np.asarray(fit['T']);delta=T[:3,3]-guess[:3,3]
                angle=math.degrees(Rotation.from_matrix(guess[:3,:3].T@T[:3,:3]).magnitude())
                result.update(innovation_m=float(np.linalg.norm(delta)),innovation_deg=angle)
                if result['innovation_m']<=self.options['max_innovation_m'] and angle<=self.options['max_innovation_deg']:
                    result.update(accepted=True,reason='LOCAL_GEOMETRY',T=T.tolist())
                else:result['reason']='OUTSIDE_LOCAL_CORRECTION'
            else:result['reason']='WEAK_MAP_GEOMETRY'
            if recorder is not None:recorder.emit(dict(kind='automatic_map_match',points_body=body.astype(np.float32),
                observation=result,local_odometry=local,map_alignment=alignment))
        except (ValueError,TypeError,KeyError,IndexError,OverflowError,np.linalg.LinAlgError) as exc:
            result['reason']=str(exc)[:160]
        self.latest=result
        return result
