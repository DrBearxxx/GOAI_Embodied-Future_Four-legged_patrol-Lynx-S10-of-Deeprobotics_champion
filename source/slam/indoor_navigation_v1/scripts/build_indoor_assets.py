"""Generate a separate first-10-m route and map subset; never edit source assets."""
import json,sys,shutil
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT,BASE
from s10nav.util import read,write,sha

def main():
    source=BASE.parent/'slam_local/runs/site_20260913_two_sessions_joint_v1/fusion_map/route_A.csv'
    a=np.genfromtxt(source,delimiter=',',names=True)
    # Use the published trajectory's metric arc; retaining rather than smoothing
    # corners avoids inventing unobserved shortcuts near furniture or walls.
    s=np.unique(np.r_[0.,a['arc_length_m'][a['arc_length_m']<10.],np.arange(.5,10.,.5),10.])
    xyz=np.column_stack([np.interp(s,a['arc_length_m'],a[k]) for k in ('x','y','z')])
    keep=[0]
    for i in range(1,len(s)-1):
        if np.linalg.norm(xyz[i]-xyz[keep[-1]])>=.25:keep.append(i)
    if np.linalg.norm(xyz[-1]-xyz[keep[-1]])<.2:keep.pop()
    keep.append(len(s)-1);xyz=xyz[keep];s=s[keep]
    direction=np.arctan2(np.diff(xyz[:,1]),np.diff(xyz[:,0]));head=np.r_[direction,direction[-1]]
    edges=[]
    for i,(p,q) in enumerate(zip(xyz[:-1],xyz[1:])):
        d=q-p;slope=float(np.degrees(np.arctan2(abs(d[2]),np.linalg.norm(d[:2]))))
        warnings=['slope_requires_review'] if slope>8 else []
        edges.append(dict(start=i,end=i+1,length_m=float(np.linalg.norm(d)),slope_deg=slope,warnings=warnings,speed_limit_mps=.10))
    route=dict(schema='s10.indoor_route.v1',frame='joint_map',source_session='A',source_sha256=sha(source),
        selection='First 10.0 m along session A recorded arc, not a 10 m radius and not the outdoor waypoint route',
        reference_end_offset_s=float(np.interp(10.,a['arc_length_m'],a['offset_s'])),
        waypoints=[dict(name=f'I{i:02d}',xyz=p.tolist(),yaw=float(y),source_arc_m=float(d)) for i,(p,y,d) in enumerate(zip(xyz,head,s))],
        edges=edges,stop_at_end=True,automatic_return=False,indoor_classification='user-provided; physical route clearance not yet checked',
        original_66_waypoints_modified=False)
    assets=ROOT/'assets';assets.mkdir(exist_ok=False)
    write(assets/'route.json',route)
    full=BASE/'assets';m=np.load(full/'map.npy');z=np.load(full/'normals.npz')
    # Keep a generous 3-D 25 m neighborhood for matching and occluding surfaces.
    dist=cKDTree(xyz).query(m,workers=4)[0];mask=dist<25
    np.save(assets/'map.npy',m[mask]);np.savez_compressed(assets/'normals.npz',normals=z['normals'][mask],valid=z['valid'][mask])
    g=np.load(full/'gallery.npz');gd=cKDTree(xyz).query(g['poses'][:,:3,3])[0]
    gm=(gd<4)&(abs(g['poses'][:,2,3]-np.median(xyz[:,2]))<.6)
    np.savez_compressed(assets/'gallery.npz',**{k:g[k][gm] for k in g.files})
    shutil.copyfile(full/'extrinsics.json',assets/'extrinsics.json')
    cfg=read(BASE/'config.json');cfg['mode']='indoor_guarded_trial';cfg['registration']['local_radius_m']=25.
    cfg['route'].update(max_shadow_speed_mps=.1,max_yaw_rate_radps=.20,goal_radius_m=.20,corridor_half_width_m=.45,height_tolerance_m=.22,lookahead_m=.4)
    cfg['notes']=['Separate first 10 m trial; operator clearance and SDK ownership checks required.',
        'Online localization thresholds are unchanged; no absolute-accuracy claim.',
        'Dual healthy LiDARs required for motor release; partial data can still localize.']
    write(ROOT/'config.json',cfg)
    original=read(full/'manifest.json')
    write(assets/'manifest.json',dict(schema='s10.indoor_assets.v1',map_id=original['map_id'],map_sha256=original['map_sha256'],
        parent_manifest_sha256=sha(full/'manifest.json'),points=int(mask.sum()),gallery_entries=int(gm.sum()),
        route_source=route['selection'],files={p.name:sha(p) for p in assets.iterdir()},absolute_accuracy_verified=False))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,6));bg=m[mask];bg=bg[(bg[:,2]>-.4)&(bg[:,2]<1.8)][::20]
    ax.scatter(bg[:,0],bg[:,1],s=.3,c='#aaaaaa');ax.plot(xyz[:,0],xyz[:,1],'-o',ms=3,c='#007c91')
    ax.scatter(*xyz[0,:2],c='green',s=80,label='Recording start');ax.scatter(*xyz[-1,:2],c='red',s=80,label='10 m arc / STOP')
    for i in [0,len(xyz)//2,len(xyz)-1]:ax.annotate(f'I{i:02d}',xyz[i,:2],xytext=(5,8),textcoords='offset points')
    ax.set(xlim=(-2,10),ylim=(-4,5),xlabel='Map X (m)',ylabel='Map Y (m)',title='Indoor trial: first 10 m of session A; no automatic return')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(ROOT/'indoor_route.png',dpi=150)
    print(json.dumps(dict(points=len(xyz),end=xyz[-1].tolist(),recording_end_s=route['reference_end_offset_s'],z_range=float(np.ptp(xyz[:,2])),map_points=int(mask.sum()),gallery=int(gm.sum()))))
if __name__=='__main__':main()
