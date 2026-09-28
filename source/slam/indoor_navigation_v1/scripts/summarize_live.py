import argparse,collections,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--run');a=p.parse_args();d=Path(a.directory)
f=d/f'health-{a.run}.jsonl' if a.run else sorted(d.glob('health-*.jsonl'))[-1]
h=[json.loads(x) for x in f.read_text().splitlines()];start=h[0]['mono'];steady=[x for x in h if x['mono']-start>=5.]
def stats(v):
    v=[x for x in v if x is not None]
    return dict(n=len(v),median=float(np.median(v)),p95=float(np.quantile(v,.95)),max=float(max(v))) if v else dict(n=0)
def fraction(v):
    return dict(ticks=len(v),valid=sum(x['valid'] for x in v)/max(1,len(v)),dual_valid=sum(bool(x['valid'] and not x['snapshot']['health']['single_lidar']) for x in v)/max(1,len(v)),
        dual_perception_and_pose=sum(bool(x['valid'] and not x['snapshot']['health']['single_lidar'] and x['snapshot']['front_fresh'] and x['snapshot']['rear_fresh']) for x in v)/max(1,len(v)))
snap=[x['snapshot'] for x in steady if x['snapshot']];pos=np.array([s['pose'][:3] for s in snap])
report=dict(run=f.stem,seconds=h[-1]['mono']-start,all=fraction(h),after_5s=fraction(steady),
    queue_drops=h[-1]['queue_drops'],ingest_counts=h[-1]['ingest_counts'],errors=sorted(set(x['error'] for x in h if x['error'])),
    pose_age_s=stats([s['health']['age_s'] for s in snap]),generations=sorted(set(s['generation'][0] for s in snap)),
    pose_spread_from_median_m=stats(np.linalg.norm(pos-np.median(pos,axis=0),axis=1).tolist()) if len(pos) else {},
    pipeline={k:stats([x['pipeline'][k] for x in steady if x.get('pipeline') and x['pipeline'][k] is not None]) for k in ('queue_ms','convert_ms','copy_ms','engine_ms','measurement_age_ms')},
    scope='Stationary live availability and repeatability, NOT absolute accuracy; robot commands disabled')
(d/(f.stem+'-summary.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
