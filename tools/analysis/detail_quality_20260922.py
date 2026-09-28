from pathlib import Path
from collections import Counter,defaultdict
from datetime import datetime,timezone,timedelta
import csv,json,math

out=Path(__file__).resolve().parent.parent/'outputs/nav-quality-20260922'
root=out/'nav-quality-20260922-pe7t26um'
rows=[json.loads(line) for line in (out/'samples.jsonl').open(encoding='utf-8')]
events=json.loads((out/'events.json').read_text(encoding='utf-8'))
state=json.loads((root/'state.json').read_text(encoding='utf-8'))
current=json.loads((root/'private/route-calibration.json').read_text(encoding='utf-8'))
old=json.loads((out.parent/'lio-comparison-20260921/20260921-205016-live-front-nav-comparison/navigation_evidence/source/private/route-calibration.json').read_text(encoding='utf-8'))
def clock(ns):return datetime.fromtimestamp(ns/1e9,timezone(timedelta(hours=8))).strftime('%H:%M:%S.%f')[:12]
def wrap(a):return (a+math.pi)%(math.pi*2)-math.pi
def summary(values):
    v=sorted(x for x in values if x is not None and math.isfinite(x))
    return dict(n=len(v),p50=v[int((len(v)-1)*.5)] if v else None,p95=v[int((len(v)-1)*.95)] if v else None,max=max(v) if v else None)
durations=defaultdict(float)
for a,b in zip(rows,rows[1:]):durations[a['loc'].get('mode')]+=min(.5,b['t']-a['t'])
auto=[r for r in rows if r['nav'].get('owner')=='auto']
following=[r for r in auto if r['nav'].get('tracking')]
print('TRACK_EXAMPLE',json.dumps(following[0]['nav'].get('tracking'),ensure_ascii=False))
alignments=[]
for a,b in zip(rows,rows[1:]):
    if a['loc'].get('generation')!=b['loc'].get('generation'):
        pa,pb=a['loc']['pose'],b['loc']['pose']
        alignments.append(dict(time=clock(b['wall_ns']),xy_change_m=math.dist(pa[:2],pb[:2]),z_change_m=pb[2]-pa[2],yaw_change_deg=math.degrees(wrap(pb[3]-pa[3])),dt_s=b['t']-a['t'],before=pa,after=pb,map_quality=b.get('map_metrics')))
changed=[];catalog=state['mission']['catalog'];points=catalog['full']['waypoints'];by_id={p['id']:p for p in points}
for pid,value in current['active']['full'].get('points',{}).items():
    previous=old['active'].get('full',{}).get('points',{}).get(pid)
    if previous is None or value['xyz']!=previous['xyz']:
        changed.append(dict(id=pid,name=by_id[pid]['name'],xyz=value['xyz'],delta_m=None if previous is None else math.dist(value['xyz'],previous['xyz']),capture=value.get('capture')))
legs=[]
for i,(a,b) in enumerate(zip(points,points[1:])):
    controls=catalog['full']['edges'][i].get('control_points',[]);chain=[a['xyz'],*[p['xyz'] for p in controls],b['xyz']]
    legs.append(dict(start=a['name'],end=b['name'],length_m=sum(math.dist(a,b) for a,b in zip(chain,chain[1:])),policy=catalog['full']['edges'][i]['policy']))
stops=Counter(r['nav']['state'] for r in auto)
tracking_by_target=[]
for name in dict.fromkeys(r['nav']['target_name'] for r in following):
    subset=[r for r in following if r['nav']['target_name']==name]
    tracking_by_target.append(dict(target=name,frames=len(subset),cross_track=summary([math.hypot(*r['nav']['tracking']['heading_trajectory']['cross_track_vector']) for r in subset]),vx=summary([abs(r['nav']['vx']) for r in subset]),vy=summary([abs(r['nav']['vy']) for r in subset]),wz=summary([abs(r['nav']['wz']) for r in subset])))
# Stationary reporting: only repeated fresh samples with measured body speed below 0.03 m/s.
last=[r for r in rows if rows[-1]['t']-r['t']<=300 and r['loc'].get('pose')]
stationary=None
if last:
    a,b=last[0],last[-1];stationary=dict(duration_s=b['t']-a['t'],xy_change_m=math.dist(a['loc']['pose'][:2],b['loc']['pose'][:2]),yaw_change_deg=math.degrees(wrap(b['loc']['pose'][3]-a['loc']['pose'][3])),measured_speed=summary([math.hypot(*(r['odom'].get('velocity',[0,0])[:2])) for r in last]))
report=dict(window=[clock(rows[0]['wall_ns']),clock(rows[-1]['wall_ns'])],samples=len(rows),mode_durations_s=dict(durations),
    odom_age_s=summary([r['loc'].get('odom_age_s') for r in rows]),epochs=sorted(set(str(r['odom'].get('epoch')) for r in rows)),
    manual_alignment_changes=alignments,calibration_revision=current['revision'],baseline_revision=old['revision'],
    draft_dirty=current['active']!=current['draft'],changed_points=changed,route_legs=legs,
    auto_states=dict(stops),tracking_by_target=tracking_by_target,stationary_tail=stationary)
(out/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
with (out/'trajectory.csv').open('w',encoding='utf-8',newline='') as f:
    w=csv.writer(f);w.writerow(['wall_time','mono','x','y','z','yaw','owner','target','state','age_s','generation','cross_track','vx','vy','wz'])
    for r in rows[::2]:
        loc=r['loc'];nav=r['nav'];p=loc.get('pose') or [None]*4;track=nav.get('tracking',{}).get('heading_trajectory',{}).get('cross_track_vector',[])
        w.writerow([clock(r['wall_ns']),r['t'],*p,nav.get('owner'),nav.get('target_name'),nav.get('state'),loc.get('odom_age_s'),loc.get('generation'),math.hypot(*track) if track else None,nav.get('vx'),nav.get('vy'),nav.get('wz')])
print(json.dumps({k:v for k,v in report.items() if k not in ('changed_points','route_legs','tracking_by_target')},ensure_ascii=False,indent=2))
print('CHANGED',[(p['name'],round(p['delta_m'],3) if p['delta_m'] is not None else None) for p in changed])
print('TRACKING',json.dumps(tracking_by_target,ensure_ascii=False))
print('LEGS_LONG',sorted(legs,key=lambda x:x['length_m'],reverse=True)[:8])
