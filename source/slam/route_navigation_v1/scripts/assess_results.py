"""Post-hoc diagnostics only: loads map trajectory AFTER predictions are frozen."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,write,sha,stats

def assess(directory):
    summary=read(directory/'summary.json');session=summary['session'];source=ROOT.parent/'slam_local/runs'
    manifest=read(ROOT/f'replay_data/{session}/manifest.json');repaired=Path(manifest['source']).parent
    before=sha(directory/'trajectory.csv');a=np.genfromtxt(directory/'trajectory.csv',delimiter=',',names=True)
    a=np.atleast_1d(a);a=a[np.isfinite(a['t'])] if a.size else a
    comparison={};aligned=None
    if len(a):
        reference=np.genfromtxt(source/'site_20260913_two_sessions_joint_v1'/f'fusion_map/route_{session}.csv',delimiter=',',names=True)
        cache=np.load(repaired/'imu_clock_samples.npz')['front'];native=cache[:,1].copy();clock=read(repaired/'clock_analysis.json')
        for jump in clock['sensors']['front']['jumps']:native[jump['index']:]-=jump['removed_clock_step_s']
        t=np.interp(a['t']+manifest['t0_host'],cache[:,0],native)
        ok=(t>=reference['timestamp'][0])&(t<=reference['timestamp'][-1]);t=t[ok];p=np.c_[a['x'],a['y'],a['z']][ok]
        ref=np.column_stack([np.interp(t,reference['timestamp'],reference[k]) for k in ('x','y','z')])
        R=Slerp(reference['timestamp'],Rotation.from_quat(np.c_[reference['qx'],reference['qy'],reference['qz'],reference['qw']]))(t)
        pred=Rotation.from_quat(np.c_[a['qx'],a['qy'],a['qz'],a['qw']][ok]);rot=np.rad2deg((R.inv()*pred).magnitude())
        dist=np.linalg.norm(p-ref,axis=1);valid=a['valid'][ok]>0
        comparison=dict(aligned=len(t),valid_aligned=int(valid.sum()),position_difference_m=stats(dist[valid]),rotation_difference_deg=stats(rot[valid]),
            confirmed_over_0p5m_or_5deg=int(np.count_nonzero(valid&((dist>.5)|(rot>5)))),
            confirmed_position_over_0p5m=int(np.count_nonzero(valid&(dist>.5))),confirmed_rotation_over_5deg=int(np.count_nonzero(valid&(rot>5))),
            largest_position_differences=[dict(t=float(a['t'][ok][i]),difference_m=float(dist[i]),rotation_deg=float(rot[i]),prediction_xyz=p[i].tolist(),reference_xyz=ref[i].tolist()) for i in np.flatnonzero(valid)[np.argsort(dist[valid])[-5:][::-1]]],
            diagnostic_thresholds_only=True,reference_is_shared_map_trajectory_not_ground_truth=True,
            offline_clock_correction_used_only_for_posthoc_audit=True)
        aligned=(a['t'][ok],dist,rot,valid,ref)
    health=[json.loads(s) for s in (directory/'health.jsonl').read_text().splitlines()]; htime=np.array([h['t'] for h in health])
    valid=np.array([h['localization']['valid'] for h in health]);fault_checks=[]
    for rule in summary['fault_rules']:
        if rule['sensor']=='all' and rule['action']=='drop' and 'kind' not in rule:
            mask=(htime>=rule['start']+1)&(htime<rule['end'])
            fault_checks.append(dict(rule=rule,sampled=int(mask.sum()),valid_after_deadline=int(valid[mask].sum()),
                nonzero_shadow_after_deadline=sum(bool(health[i]['vx'] or health[i]['wz']) for i in np.flatnonzero(mask))))
    result=dict(prediction_sha256=before,reference_audit=comparison,all_sensor_outage_checks=fault_checks,
                autonomous_release_passed=False,release_reason='Independent walk, physical corridor/terrain validation and robot-level stop tests still required.')
    write(directory/'assessment.json',result);assert sha(directory/'trajectory.csv')==before
    fig,axes=plt.subplots(3,1,figsize=(13,9));axes[0].plot(htime,valid.astype(int),lw=.7,color='#117a87');axes[0].set_ylabel('Localization valid');axes[0].set_ylim(-.1,1.1)
    if aligned:
        t,d,r,v,_=aligned;axes[1].plot(t[v],d[v],'.',ms=2);axes[1].axhline(.5,color='r',ls='--');axes[1].set_ylabel('Map-reference difference (m)')
        axes[2].plot(t[v],r[v],'.',ms=2);axes[2].axhline(5,color='r',ls='--');axes[2].set_ylabel('Rotation difference (deg)')
    for ax in axes:
        ax.grid(alpha=.2)
        for rule in summary['fault_rules']:ax.axvspan(rule['start'],rule['end'],alpha=.06,color='red')
    axes[-1].set_xlabel('Recorded reception time since first selected event (s)')
    fig.suptitle(f'{session} / {summary["scenario"]} - causal replay, shared-map diagnostic (NOT ground truth)');fig.tight_layout();fig.savefig(directory/'validation.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(directory=str(directory),**result)),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();assess(Path(a.directory))
