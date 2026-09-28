"""A clicked location narrows retrieval, never supplies an accepted pose."""
import numpy as np
import math
from scipy.spatial.transform import Rotation
from indoor.pipeline import IndoorMatcher
from s10nav.engine import Engine
from localization_quality import classify

class AppEngine(Engine):
    """A failing rear scan must not consume the front sensor's retrieval turn."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.retrieval_times = {}

    def event(self,event,now=None):
        sensor=event.get('sensor')
        retrieval=event['kind']=='cloud' and self.T is None
        if retrieval:self.last_global=self.retrieval_times.get(sensor,-1e9)
        result=super().event(event,now)
        if retrieval:self.retrieval_times[sensor]=self.last_global
        if result and result.get('accepted') and hasattr(self.motion,'observe_lidar'):
            self.motion.observe_lidar(self.last_t,self.T)
        return result

    def poll(self,now):
        health=super().poll(now)
        if health['valid'] and self.last_metrics.get('quality_level')=='degraded':health['state']='DEGRADED'
        return health

class SeedMatcher(IndoorMatcher):
    def __init__(self, assets, cfg):
        super().__init__(assets, cfg)
        self.full_gallery = self.gallery
        self.scope = None

    def begin(self, request):
        xyz = np.asarray(request['xyz'])
        positions = self.full_gallery['poses'][:, :3, 3]
        keep = ((np.linalg.norm(positions[:, :2]-xyz[:2], axis=1) <= request['radius']) &
                (abs(positions[:, 2]-xyz[2]) <= request['height_tolerance']))
        if not np.any(keep): raise ValueError('NO_REFERENCE_KEYFRAMES_NEAR_POINT')
        self.gallery = {k:v[keep] for k,v in self.full_gallery.items()}
        self.scope = request
        return int(keep.sum())

    def finish(self):
        self.scope = None
        self.gallery = self.full_gallery

    def register(self, points, initial, wide=False):
        result = super().register(points, initial, wide)
        q=result.get('quality') or {};warnings=result.get('reasons',[])
        level=classify(q.get('overlap'),q.get('median_nn_m'),q.get('condition'),'not_converged' not in warnings)
        result['quality_level']=level;result['quality_warnings']=list(warnings)
        if level=='unusable':result.update(accepted=False,reasons=warnings or ['geometry_unusable'])
        if level!='unusable' and result.get('T') is not None:
            result.update(accepted=True,reasons=[],verification='usable_heldout_geometry')
        if result['accepted'] and self.scope is not None:
            d = result['T'][:3, 3]-np.array(self.scope['xyz'])
            if np.linalg.norm(d[:2]) > self.scope['radius'] or abs(d[2]) > self.scope['height_tolerance']:
                result = {**result, 'accepted':False, 'reasons':['FIT_OUTSIDE_SELECTED_REGION']}
        if result.get('T') is not None:
            result['pose']=[*map(float,result['T'][:3,3]),float(np.arctan2(result['T'][1,0],result['T'][0,0]))]
        if result.get('accepted') and self.scope is not None and 'heading_prior' in self.scope:
            # Resolve near-equal geometric alternatives using continuous
            # heading. This is a soft score term, not a new confidence gate.
            error=(result['pose'][3]-self.scope['heading_prior']+math.pi)%(2*math.pi)-math.pi
            penalty=.04*(1-math.cos(error))
            result.update(geometric_score=result['score'],heading_prior=self.scope['heading_prior'],
                          heading_prior_error_rad=error,heading_prior_penalty=penalty)
            result['score']-=penalty
        return result

    def relocalize(self,points,sensor,Rlevel,registration_points=None):
        hints=[]
        if self.scope is not None and 'heading_prior' in self.scope:
            # A drifted heading is a search center, not a fixed orientation.
            # Adjacent basins matter on repeated stairs/landings where the
            # descriptor's small top-k shortlist can miss the correct basin.
            for delta in (-math.pi/6,0.,math.pi/6):
                initial=np.eye(4);initial[:3,3]=self.scope['xyz']
                initial[:3,:3]=Rotation.from_euler('z',self.scope['heading_prior']+delta).as_matrix()@Rlevel
                hints.append(self.register(points if registration_points is None else registration_points,initial,wide=True))
        blind=super().relocalize(points,sensor,Rlevel,registration_points)
        alternatives=[r for r in (*hints,blind) if r and r.get('accepted')]
        if not alternatives:return blind
        best=max(alternatives,key=lambda r:r['score'])
        diagnostics=list(blind.get('candidates',[]))
        for hinted in hints:diagnostics.append(dict(candidate_source='continuous_heading',**{k:v for k,v in hinted.items() if k!='T'}))
        return {**best,'candidates':diagnostics,'candidate_count':len(diagnostics),
                'heading_prior_used':bool(hints)}
