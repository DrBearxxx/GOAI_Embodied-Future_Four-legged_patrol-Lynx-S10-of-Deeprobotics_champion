"""Post-process measured standing test; no ROS/robot writes."""
import collections,json,sys
from pathlib import Path
import numpy as np
from audit_goai_route import audit

root=Path(__file__).resolve().parents[1];out=Path(sys.argv[1])
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def lines(p):return [json.loads(s) for s in p.read_text(encoding='utf-8').splitlines() if s.strip()]
def stats(a):
    a=np.array(a,float)
    return dict(min=float(np.min(a)),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(np.max(a))) if len(a) else None
summary=read(out/'summary.json');tele=lines(out/'telemetry.jsonl');observed=[r for r in tele if r['phase']=='standing_observation']
assert observed
begin=observed[0]['mono'];end=observed[-1]['mono']
route=lines(next((out/'raw').glob('goai-route-*.jsonl')))
health=[r for r in lines(next((out/'raw').glob('health-*.jsonl'))) if begin<=r['mono']<=end]
matches=lines(next((out/'raw').glob('localize-*.jsonl')))
solutions=[r['solution'] for r in health if r.get('solution')]
poses=np.array([r['pose'] for r in solutions]);modes=collections.Counter(s['mode'] for s in solutions)
goal=np.array(read(root/'assets/route.json')['waypoints'][0]['xyz'])
median=np.median(poses,axis=0);dist=np.linalg.norm(poses[:,:3]-goal,axis=1)
displacement=np.linalg.norm(poses[:,:3]-median[:3],axis=1)
native=[r['status'] for r in observed]
all_clear=[all(s in r['safety_records'] and r['safety_records'][s]['valid'] is True and not r['safety_records'][s]['blocked'] for s in ('front','rear')) for r in health]
both_fresh=[all(s in r['safety_records'] and 0<=r['mono']-r['safety_records'][s]['mono']<=.25 for s in ('front','rear')) for r in health]
report=dict(seconds=end-begin,route_audit=audit(route),samples=len(health),
    modes=dict(modes),estimate_available_fraction=sum(s['mode']!='LOST' for s in solutions)/len(health),
    dual_safety_fresh_fraction=sum(both_fresh)/len(health),both_clear_fraction=sum(all_clear)/len(health),
    geometry_measurement_age_s=stats([r['mono']-r['solution']['measurement_mono'] for r in health if r.get('solution')]),
    imu_estimate_age_s=stats([r['mono']-r['solution']['estimate_mono'] for r in health if r.get('solution')]),
    median_pose_xyz_yaw=median.tolist(),route_start_xyz=goal.tolist(),distance_to_route_start_m=stats(dist),
    pose_deviation_from_median_m=stats(displacement),pose_xyz_std_m=np.std(poses[:,:3],axis=0).tolist(),
    yaw_std_deg=float(np.std(np.unwrap(poses[:,3]))*180/np.pi),
    first_last_3s_pose_delta_m=(np.median(poses[-150:,:3],axis=0)-np.median(poses[:150,:3],axis=0)).tolist(),
    valid_healthy_fraction=sum(s['healthy'] is True and s['fault'] is False and s['ownership'] is True for s in native)/len(native),
    zero_selected_velocity_fraction=sum(all(v==0 for v in s['selected_velocity']) for s in native)/len(native),
    joint_age_s=stats([s['joint_age_s'] for s in native]),native_imu_age_s=stats([s['imu_age_s'] for s in native]),
    policy_ms=stats([s['policy_ms'] for s in native]),
    match_accepts=sum(r.get('accepted') is True and begin<=r.get('received',-1)<=end for r in matches),
    match_reasons=dict(collections.Counter(r.get('reason','') for r in matches if not r.get('accepted') and begin<=r.get('received',-1)<=end)),
    safety_hits={s:stats([r['safety_records'][s]['hits'] for r in health if s in r['safety_records'] and 'hits' in r['safety_records'][s]]) for s in ('front','rear')},
    input_rejects_last=health[-1].get('input_rejects'),last_ingest_counts=health[-1]['ingest_counts'],
    final_state=summary['final_goai_status']['mode'],absolute_accuracy_verified=False,
    limitations=['Static repeatability includes real zero-target policy motion; not absolute position accuracy.',
        'Obstacle identity cannot be inferred solely from LiDAR clusters.',
        'Localizer exited with duplicate rclpy shutdown error after deliberate SIGINT at end of test.'])
snapshot=read(out/'clearance_snapshot.json')
report['clearance_snapshot_during_observation']=all(begin<=r['mono']<=end for r in snapshot.values())
(out/'analysis.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2))
