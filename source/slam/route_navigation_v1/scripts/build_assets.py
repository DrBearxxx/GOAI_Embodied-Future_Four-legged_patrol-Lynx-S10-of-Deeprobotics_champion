"""Compile user waypoint order and frozen localization assets, never edit evidence."""
import argparse
import copy
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,write,sha,mat,lidar_extrinsic,voxel,cap
from s10nav.matching import descriptor

def compile_route(draft,reference,cfg):
    points=copy.deepcopy(draft['waypoints']); xyz=np.array([p['xyz'] for p in points],float)
    if len(points)<2 or not np.isfinite(xyz).all() or len({p['id'] for p in points})!=len(points): raise ValueError('Invalid waypoint list')
    ref_tree=cKDTree(reference); edges=[]; dense=[]; arc=0.
    for i,p in enumerate(points):
        d=xyz[min(i+1,len(points)-1)]-xyz[i] if i<len(points)-1 else xyz[-1]-xyz[-2]
        if np.linalg.norm(d[:2])<1e-6: raise ValueError(f'Waypoint {i+1} has no XY direction')
        p['original_yaw']=p['yaw']; p['yaw']=float(np.arctan2(d[1],d[0])); p['user_position_order_confirmed']=True
        p['yaw_policy']='toward_next_waypoint' if i<len(points)-1 else 'continue_last_segment'
        if i==len(points)-1: break
        length=float(np.linalg.norm(d)); n=max(1,int(np.ceil(length/cfg['sample_spacing_m'])))
        segment=xyz[i]+np.linspace(0,1,n+1)[:,None]*d
        distance=ref_tree.query(segment)[0]; slope=float(np.rad2deg(np.arctan2(abs(d[2]),np.linalg.norm(d[:2]))))
        warnings=[]
        if distance.max()>1.5: warnings.append('intermediate_path_away_from_recorded_corridor')
        if slope>cfg['terrain_slope_warning_deg']: warnings.append('slope_or_stairs_requires_terrain_validation')
        edges.append(dict(index=i,from_id=p['id'],to_id=points[i+1]['id'],length_m=length,start_s_m=arc,
            end_s_m=arc+length,slope_deg=slope,reference_max_distance_m=float(distance.max()),warnings=warnings,
            kind='terrain_transition' if slope>cfg['terrain_slope_warning_deg'] else 'normal',
            speed_limit_mps=cfg['max_shadow_speed_mps'],corridor_half_width_m=cfg['corridor_half_width_m'],
            physical_traversability_verified=False,interpolation='piecewise_linear_preserves_all_user_points'))
        for k,q in enumerate(segment[:-1]): dense.append([i,arc+length*k/n,*q,p['yaw']])
        arc+=length
    dense.append([len(edges)-1,arc,*xyz[-1],points[-1]['yaw']])
    return dict(schema='s10.compiled_route.v1',map_id=draft['map_id'],map_sha256=draft['map_sha256'],frame=draft['frame'],
        status='shadow_validation',autonomous_use_approved=False,user_position_order_confirmed=True,
        route_start_policy='must_reach_first_waypoint_or_explicit_operator_reassociation',
        endpoint_id=points[-1]['id'],waypoints=points,edges=edges,length_m=arc,
        warnings=['All user points and order retained; B31/B32 not merged.',
                  'Interpolation and headings implemented; traversability and corridor clearance are not certified.',
                  'Last point inherits last segment heading because there is no next point.']),np.array(dense)

def main():
    p=argparse.ArgumentParser();p.add_argument('--draft',required=True);p.add_argument('--map-run',required=True);a=p.parse_args()
    run=Path(a.map_run); assets=ROOT/'assets'; assets.mkdir(exist_ok=True); cfg=read(ROOT/'config.json')
    draft=read(a.draft); check=read(run/'fusion_map/artifact_checks.json')
    expected=check['artifacts']['map.ply']['sha256']
    if draft['map_sha256']!=expected or sha(run/'fusion_map/map.ply')!=expected: raise ValueError('Map/draft checksum mismatch')
    original_hash=sha(a.draft)
    A=np.genfromtxt(run/'fusion_map/route_A.csv',delimiter=',',names=True)
    ref=np.c_[A['x'],A['y'],A['z']]
    route,dense=compile_route(draft,ref,cfg['route'])
    route['input_sha256']=original_hash; write(assets/'route.json',route); write(assets/'original_waypoints.json',draft)
    np.savetxt(assets/'route_samples.csv',dense,delimiter=',',header='segment,s,x,y,z,yaw',comments='')
    cloud=np.load(run/'fusion_map/localization_map.npy',allow_pickle=False)
    cloud=voxel(cloud,cfg['map_voxel_m']).astype(np.float32); np.save(assets/'map.npy',cloud)
    print('MAP',len(cloud),'estimating normals',flush=True); tree=cKDTree(cloud); normals=np.zeros_like(cloud); valid=np.zeros(len(cloud),bool)
    for k in range(0,len(cloud),15000):
        pnts=cloud[k:k+15000]; d,ix=tree.query(pnts,k=20,workers=4); q=cloud[ix].astype(float); q-=q.mean(1,keepdims=True)
        e,V=np.linalg.eigh(np.einsum('nki,nkj->nij',q,q)/20)
        normals[k:k+len(pnts)]=V[:,:,0];valid[k:k+len(pnts)]=(e[:,0]/np.maximum(e.sum(1),1e-12)<.08)&(d[:,-1]<1.5)
    np.savez_compressed(assets/'normals.npz',normals=normals,valid=valid)
    poses=[]; desc=[]; levels=[]; sensors=[]; provenance=[]
    for session,source in [('A','site_20260913_131746_fusion_v1'),('B','site_20260913_143933_fusion_v1')]:
        rows=np.genfromtxt(run/f'fusion_map/route_{session}.csv',delimiter=',',names=True); previous=None; last=-100
        for i,row in enumerate(rows):
            T=mat([row[k] for k in ('x','y','z')],[row[k] for k in ('qx','qy','qz','qw')])
            if previous is not None and np.linalg.norm(T[:3,3]-previous)<3 and i-last<8: continue
            previous=T[:3,3];last=i; y=np.arctan2(T[1,0],T[0,0]); Rlevel=Rotation.from_euler('z',-y).as_matrix()@T[:3,:3]
            for sensor in ['front','rear']:
                file=run/f'cache/redeskew/{session}/{sensor}/{i:04d}.npz'
                if not file.exists():file=run.parent/source/f'cache/body/{sensor}/{i:04d}.npz'
                if not file.exists():continue
                pts=cap(np.load(file)['points'].astype(float),5000)
                if len(pts)<600:continue
                desc.append(descriptor(pts,Rlevel,cfg['relocalization']));poses.append(T);levels.append(Rlevel);sensors.append(sensor)
                provenance.append(dict(session=session,node=i,sensor=sensor))
    np.savez_compressed(assets/'gallery.npz',descriptors=np.array(desc),poses=np.array(poses),level_R=np.array(levels),sensors=np.array(sensors))
    write(assets/'gallery_provenance.json',provenance)
    base=run.parent/'site_20260913_143933_repaired_v1/recovered_source'; cal=read(base/'calibration.json')
    cam=read(run.parent/'site_20260913_143933_fusion_v1/calibration.json'); X={}
    for side in ['front','rear']:
        v=cal[side]['T_lidar_imu'];X[side]=(lidar_extrinsic(side)@mat(v[:3],v[3:])).tolist()
    X['camera']=cam['T_body_camera_imu']
    write(assets/'extrinsics.json',dict(imu=X,lidar={s:lidar_extrinsic(s).tolist() for s in ('front','rear')},
        calibration_verified=False,source='existing fixed map calibration; nominal translations; inverted camera retained',
        depth=cam['T_body_depth']))
    files=['map.npy','normals.npz','gallery.npz','route.json','extrinsics.json']
    manifest=dict(schema='s10.localization_assets.v1',map_id=draft['map_id'],map_sha256=expected,
        input_route_sha256=original_hash,points=len(cloud),gallery_entries=len(desc),
        files={name:sha(assets/name) for name in files},absolute_accuracy_verified=False,autonomous_use_approved=False,
        map_training_sessions=['A','B'],query_truth_used_by_runtime=False)
    write(assets/'manifest.json',manifest); assert sha(a.draft)==original_hash
    print(json.dumps(dict(waypoints=len(route['waypoints']),path_length_m=route['length_m'],map_points=len(cloud),gallery=len(desc),warnings=sum(bool(e['warnings']) for e in route['edges']))),flush=True)

if __name__=='__main__':main()
