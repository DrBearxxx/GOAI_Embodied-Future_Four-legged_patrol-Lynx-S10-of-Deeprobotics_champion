"""Frozen-map GICP and blind geometric place retrieval; no query reference poses."""
import sys
import time
from collections import OrderedDict
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .util import ROOT, read, apply, delta, voxel, cap

try:
    import small_gicp
except ImportError:
    sys.path.insert(0,str(ROOT.parent/'slam_local/runs/site_insight9_A_lidar_localization_v1/python_deps'))
    import small_gicp

def descriptor(points,Rlevel,cfg):
    p=np.asarray(points)@Rlevel.T; r=np.linalg.norm(p[:,:2],axis=1)
    ok=np.isfinite(p).all(axis=1)&(r>.6)&(r<cfg['radius_m']); p,r=p[ok],r[ok]
    out=np.zeros((cfg['rings'],cfg['sectors']),np.float32)
    rings=np.floor(r/cfg['radius_m']*cfg['rings']).astype(int)
    sectors=np.floor((np.arctan2(p[:,1],p[:,0])%(2*np.pi))/(2*np.pi)*cfg['sectors']).astype(int)
    np.maximum.at(out,(rings,sectors),np.maximum(p[:,2]+2,.05))
    return out

def distances(q,gallery):
    qn=q/np.maximum(np.linalg.norm(q,axis=0,keepdims=True),1e-8)
    gn=gallery/np.maximum(np.linalg.norm(gallery,axis=1,keepdims=True),1e-8)
    qm=np.linalg.norm(q,axis=0)>0; gm=np.linalg.norm(gallery,axis=1)>0; values=[]
    for shift in range(q.shape[1]):
        common=gm&np.roll(qm,shift); count=common.sum(1)
        d=1-(np.sum(gn*np.roll(qn,shift,axis=1)[None],axis=1)*common).sum(1)/np.maximum(count,1)
        d[count<12]=2.; values.append(d)
    return np.asarray(values).T

class Matcher:
    def __init__(self,assets,cfg):
        self.assets=Path(assets); self.cfg=cfg; self.g=cfg['registration']; self.cache=OrderedDict()
        self.points=np.load(self.assets/'map.npy',allow_pickle=False).astype(float)
        z=np.load(self.assets/'normals.npz',allow_pickle=False)
        self.normals=z['normals']; self.normal_valid=z['valid']; self.tree=cKDTree(self.points)
        self.plane_ids=np.flatnonzero(self.normal_valid)
        if len(self.plane_ids)<500:raise ValueError('frozen_map_has_insufficient_valid_surfels')
        self.plane_tree=cKDTree(self.points[self.plane_ids])
        self.gallery=np.load(self.assets/'gallery.npz',allow_pickle=False)

    def target(self,position):
        key=tuple(np.round(np.asarray(position)/5).astype(int)); center=np.array(key)*5
        if key in self.cache:
            self.cache.move_to_end(key); return self.cache[key]
        ix=self.tree.query_ball_point(center,self.g['local_radius_m'])
        if len(ix)<500: raise ValueError('outside_frozen_map')
        points=self.points[ix]; levels=[]
        for v in self.g['voxels_m']:
            levels.append(small_gicp.preprocess_points(points,downsampling_resolution=v,num_neighbors=20,num_threads=self.cfg['threads']))
        self.cache[key]=levels
        if len(self.cache)>6: self.cache.popitem(last=False)
        return levels

    def quality(self,p,T):
        world=apply(T,p); d,_=self.tree.query(world,workers=1)
        # Plane residuals need correspondences in the VALID SURFEL index.
        # The nearest arbitrary map point may have an unreliable normal while
        # a neighboring valid surfel describes the same physical surface.
        pd,pi=self.plane_tree.query(world,workers=1);ix=self.plane_ids[pi];valid=pd<.4
        n=self.normals[ix[valid]]; r=np.sum(n*(world[valid]-self.points[ix[valid]]),axis=1)
        rotated=p[valid]@T[:3,:3].T
        J=np.c_[np.cross(rotated,n),n]; eig=np.linalg.eigvalsh(J.T@J) if len(J) else np.zeros(6)
        rank=np.linalg.eigvalsh(np.cov(p.T)) if len(p)>3 else np.zeros(3)
        return dict(points=len(p),plane_points=int(valid.sum()),overlap=float(np.mean(d<.25)),
                    median_nn_m=float(np.median(d)),plane_rmse_m=float(np.sqrt(np.mean(r*r))) if len(r) else 1e6,
                    condition=float(max(eig[0],0)/max(eig[-1],1e-12)),source_rank=float(max(rank[0],0)/max(rank[-1],1e-12)))

    def register(self,points,initial,wide=False):
        begin=time.perf_counter(); T=np.array(initial,copy=True); fit=points[::2]; check=points[1::2]
        if len(check)<self.g['min_points']: return dict(accepted=False,reasons=['insufficient_points'])
        if not np.isfinite(points).all() or not np.isfinite(initial).all():return dict(accepted=False,reasons=['nonfinite_input'])
        try:
            levels=self.target(T[:3,3]); converged=False
            for v,maxdist,(target,tree) in zip(self.g['voxels_m'],self.g['correspondence_m'],levels):
                source,_=small_gicp.preprocess_points(np.asarray(fit,float),downsampling_resolution=v,num_neighbors=20,num_threads=self.cfg['threads'])
                result=small_gicp.align(target,source,tree,init_T_target_source=T,registration_type='GICP',
                    max_correspondence_distance=maxdist if wide else min(maxdist,.8),num_threads=self.cfg['threads'],
                    max_iterations=self.g['max_iterations'],rotation_epsilon=np.deg2rad(.05),translation_epsilon=.001)
                T=np.asarray(result.T_target_source); converged=bool(result.converged)
                if not np.isfinite(T).all(): raise ValueError('nonfinite_registration')
            q=self.quality(check,T); reasons=[]
            for condition,name in [(q['plane_points']<self.g['min_plane_points'],'insufficient_plane_support'),
                (q['overlap']<self.g['overlap_min'],'low_overlap'),(q['median_nn_m']>self.g['median_nn_max_m'],'large_nn_residual'),
                (q['plane_rmse_m']>self.g['plane_rmse_max_m'],'large_plane_residual'),
                (q['condition']<self.g['condition_min'],'degenerate_geometry'),(q['source_rank']<self.g['source_rank_min'],'rank_deficient_source'),
                (not converged,'not_converged')]:
                if condition: reasons.append(name)
            verification='heldout_surfel_geometry';split=None
            # Dense maps need not have planar neighborhoods everywhere. When
            # ONLY the surfel support test fails, use an independent second
            # GICP fit plus stricter all-point residuals, not a lower plane
            # threshold. Both disjoint fits must agree in 6 DoF.
            if reasons and set(reasons)<= {'insufficient_plane_support','large_plane_residual'} and q['overlap']>=.9 and q['median_nn_m']<=.12:
                target,tree=levels[-1]
                second,_=small_gicp.preprocess_points(np.asarray(check,float),downsampling_resolution=self.g['voxels_m'][-1],num_neighbors=20,num_threads=self.cfg['threads'])
                check_result=small_gicp.align(target,second,tree,init_T_target_source=np.array(initial),registration_type='GICP',
                    max_correspondence_distance=.8,num_threads=self.cfg['threads'],max_iterations=self.g['max_iterations'],
                    rotation_epsilon=np.deg2rad(.05),translation_epsilon=.001)
                T2=np.asarray(check_result.T_target_source);difference=delta(T,T2);q2=self.quality(fit,T2)
                split=dict(converged=bool(check_result.converged),difference_m=difference[0],difference_deg=difference[1],quality=q2)
                if check_result.converged and difference[0]<.08 and difference[1]<1.5 and q2['overlap']>=.9 and q2['median_nn_m']<=.12 and q2['condition']>=self.g['condition_min']:
                    reasons=[];verification='strict_disjoint_gicp_consistency'
            correction=delta(initial,T)
            if not wide and (correction[0]>self.g['correction_max_m'] or correction[1]>self.g['correction_max_deg']): reasons.append('prediction_disagreement')
            return dict(accepted=not reasons,T=T,reasons=reasons,quality=q,correction=list(correction),
                        score=q['overlap']-2*q['median_nn_m']-q['plane_rmse_m'],seconds=time.perf_counter()-begin,
                        verification=verification,split_verification=split)
        except (ValueError,RuntimeError,np.linalg.LinAlgError) as e:
            return dict(accepted=False,reasons=[str(e)],seconds=time.perf_counter()-begin)

    def relocalize(self,points,sensor,Rlevel,registration_points=None):
        c=self.cfg['relocalization']; sides=self.gallery['sensors']; ids=np.flatnonzero(sides==sensor)
        if not len(ids): return dict(accepted=False,reasons=['no_matching_sensor_gallery'])
        D=distances(descriptor(points,Rlevel,c),self.gallery['descriptors'][ids]); chosen=[]
        for flat in np.argsort(D.ravel()):
            i,shift=np.unravel_index(flat,D.shape); gi=ids[i]
            T=self.gallery['poses'][gi].copy(); Rg=self.gallery['level_R'][gi]
            T[:3,:3]=T[:3,:3]@Rg.T@Rotation.from_euler('z',shift*2*np.pi/c['sectors']).as_matrix()@Rlevel
            if any(delta(T,v)[0]<2 and delta(T,v)[1]<25 for v in chosen): continue
            chosen.append(T)
            if len(chosen)>=c['top_candidates']: break
        fit_points=points if registration_points is None else registration_points
        results=[self.register(fit_points,T,wide=True) for T in chosen]
        diagnostics=[dict(initial_xyz=T[:3,3].tolist(),**{k:v for k,v in r.items() if k!='T'}) for T,r in zip(chosen,results)]
        accepted=sorted([r for r in results if r['accepted']],key=lambda r:r['score'],reverse=True)
        if not accepted: return dict(accepted=False,reasons=['global_geometry_rejected'],candidate_count=len(results),candidates=diagnostics)
        best=accepted[0]
        for r in accepted[1:]:
            d=delta(best['T'],r['T'])
            if (d[0]>c['distinct_m'] or d[1]>c['distinct_deg']) and best['score']-r['score']<c['ambiguity_score_gap']:
                return dict(accepted=False,reasons=['ambiguous_global_candidates'],candidate_count=len(results),candidates=diagnostics)
        return {**best,'candidate_count':len(results),'candidates':diagnostics}
