"""Post-hoc indoor-window audit; does not modify predicted trajectories."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT,BASE
from s10nav.util import read,write,stats,sha

rows=[]
for name in ['A_final','B_final','A_faults_final']:
    d=ROOT/'results'/name;summary=read(d/'summary.json');session=summary['session'];f=d/'trajectory.csv';before=sha(f)
    p=np.genfromtxt(f,delimiter=',',names=True);manifest=read(BASE/'replay_data'/session/'manifest.json')
    source=Path(manifest['source']).parent;clock=read(source/'clock_analysis.json');c=np.load(source/'imu_clock_samples.npz')['front'];native=c[:,1].copy()
    for jump in clock['sensors']['front']['jumps']:native[jump['index']:]-=jump['removed_clock_step_s']
    t=np.interp(p['t']+manifest['t0_host'],c[:,0],native)
    ref=np.genfromtxt(BASE.parent/'slam_local/runs/site_20260913_two_sessions_joint_v1/fusion_map'/f'route_{session}.csv',delimiter=',',names=True)
    arc=np.interp(t,ref['timestamp'],ref['arc_length_m'])
    ok=(p['valid']>0)&(t>=ref['timestamp'][0])&(t<=ref['timestamp'][-1])&(arc<=10.)
    xyz=np.c_[p['x'],p['y'],p['z']][ok];target=np.c_[[np.interp(t[ok],ref['timestamp'],ref[k]) for k in ('x','y','z')]].T
    R=Slerp(ref['timestamp'],Rotation.from_quat(np.c_[ref['qx'],ref['qy'],ref['qz'],ref['qw']]))(t[ok])
    pred=Rotation.from_quat(np.c_[p['qx'],p['qy'],p['qz'],p['qw']][ok]);angle=np.degrees((R.inv()*pred).magnitude())
    item=dict(run=name,session=session,full_35s_valid_fraction=summary['valid_fraction'],first_10m_valid_positions=stats(np.linalg.norm(xyz-target,axis=1)),
        first_10m_rotation_deg=stats(angle),sampled_reception_time_s=[float(p['t'][ok].min()),float(p['t'][ok].max())],
        reference='same-map optimized trajectory, NOT independent truth',processing=summary['processing_seconds'])
    rows.append(item);assert before==sha(f)
live=[]
for f in sorted((ROOT/'results/orin_live').glob('localize-*.jsonl')):
    events=[json.loads(x) for x in f.read_text().splitlines()];accepted=[e for e in events if e.get('accepted')]
    live.append(dict(file=f.name,observations=len(events),accepted=len(accepted),valid_accepted=sum(e['health']['valid'] for e in accepted),
        imu_gap_rejects=sum(e.get('reason')=='no_scan_imu_coverage' for e in events),
        last_pose=accepted[-1]['pose'] if accepted else None,processing=stats([e['metrics']['seconds'] for e in accepted])))
for f in sorted((ROOT/'results/orin_live').glob('health-*.jsonl')):
    h=[json.loads(x) for x in f.read_text().splitlines()];snap=[x for x in h if x['snapshot']];dual=[x for x in snap if x['valid'] and not x['snapshot']['health']['single_lidar']]
    live.append(dict(file=f.name,ticks=len(h),valid_fraction=sum(x['valid'] for x in h)/max(1,len(h)),dual_valid_ticks=len(dual),
        error=h[-1]['error'] if h else None,queue_drops=h[-1]['queue_drops'] if h else None,ingest_counts=h[-1]['ingest_counts'] if h else None))
write(ROOT/'results/indoor_audit.json',dict(replays=rows,live=live,physical_navigation_completed=False,independent_accuracy_verified=False))
print(json.dumps(dict(replays=rows,live=live),indent=2))
