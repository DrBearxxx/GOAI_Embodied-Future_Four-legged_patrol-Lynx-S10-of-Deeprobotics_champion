from pathlib import Path
from collections import Counter,defaultdict
from datetime import datetime,timezone,timedelta
import tarfile,hashlib,json,math,statistics

OUT=Path(__file__).resolve().parent.parent/'outputs/nav-quality-20260922'
archive=OUT/'nav-quality-20260922-pe7t26um.tar.gz'
root=OUT/archive.name.removesuffix('.tar.gz')
if not root.exists():
    assert hashlib.sha256(archive.read_bytes()).hexdigest()=='1df154265ef95d23c94d40ce30fe0ab58732556006b1fe52254e71b56a5f447d'
    with tarfile.open(archive) as t:t.extractall(OUT,filter='data')
state=json.loads((root/'state.json').read_text(encoding='utf-8'))
cal=json.loads((root/'private/route-calibration.json').read_text(encoding='utf-8'))
print('STATE',json.dumps({k:state[k] for k in ('pose','mode','local_odometry')},ensure_ascii=False))
print('CALIBRATION',cal['revision'],{k:len(v.get('points',{})) for k,v in cal['active'].items()})
def stamp(ns):return datetime.fromtimestamp(ns/1e9,timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')
def quantile(v,p):
    v=sorted(x for x in v if x is not None and math.isfinite(x))
    return v[min(len(v)-1,int((len(v)-1)*p))] if v else None
def summary(v):return dict(n=len(v),p50=quantile(v,.5),p95=quantile(v,.95),max=max(v) if v else None)
samples=[];events=[];seen=set();example=None
for path in sorted((root/'logs').glob('navigation-trace.jsonl*')):
    with path.open(encoding='utf-8') as f:
        for line in f:
            try:v=json.loads(line)
            except ValueError:continue
            key=(v.get('log_session'),v.get('mono'),v.get('event'))
            if key in seen:continue
            seen.add(key)
            if v['event']!='navigation_tick':events.append(v);continue
            example=example or v
            loc=v.get('localization') or {};nav=v.get('navigation') or {};odom=v.get('local_odometry') or {}
            samples.append(dict(t=v['mono'],wall_ns=v['wall_ns'],session=v['log_session'],loc=loc,nav=nav,odom=odom,
                                map_metrics=v.get('map_metrics'),map_anchor=v.get('map_anchor'),control=v.get('control')))
samples.sort(key=lambda v:v['t']);events.sort(key=lambda v:v['mono'])
(OUT/'example.json').write_text(json.dumps(example,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'events.json').write_text(json.dumps(events,ensure_ascii=False,indent=2),encoding='utf-8')
print('KEYS',list(example),'NAV',list(example['navigation']),'ODOM',list(example['local_odometry']))
print('LOC_EXAMPLE',json.dumps(example['localization'],ensure_ascii=False))
print('NAV_EXAMPLE',json.dumps(example['navigation'],ensure_ascii=False))
print('TIME',stamp(samples[0]['wall_ns']),stamp(samples[-1]['wall_ns']),len(samples))
print('EVENTS',Counter(v['event'] for v in events))
for v in events:
    if v['event'] in ('operator_action','operator_alignment_committed','odometry_epoch'):
        print('ACTION',stamp(v['wall_ns']),v.get('action',v['event']),v.get('route_id'),v.get('target_index'),
              (v.get('calibration') or {}).get('revision'),(v.get('calibration_capture') or {}).get('measurement_mono'))
owners=Counter(v['nav'].get('owner') for v in samples);modes=Counter(v['loc'].get('mode') for v in samples)
print('OWNERS',owners,'MODES',modes)
print('ODOM_AGE',summary([v['loc']['odom_age_s'] for v in samples if v['loc'].get('odom_age_s') is not None]))
print('GENERATIONS',Counter(str(v['loc'].get('generation')) for v in samples))
print('MAP_AGES',summary([v['loc']['map_age_s'] for v in samples if v['loc'].get('map_age_s') is not None]))
print('NAV_STATES',Counter(v['nav'].get('state') for v in samples if v['nav'].get('owner')=='auto'))
with (OUT/'samples.jsonl').open('w',encoding='utf-8') as f:
    for v in samples:f.write(json.dumps(v,ensure_ascii=False,separators=(',',':'))+'\n')
