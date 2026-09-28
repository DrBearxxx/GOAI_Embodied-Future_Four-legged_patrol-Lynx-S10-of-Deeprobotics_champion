"""Read-only health log window summary; repeatability is not ground-truth accuracy."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--start-mono', type=float, required=True)
    p.add_argument('--end-mono', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    counters = {k: Counter() for k in ('solution_mode', 'solution_reason', 'map_state', 'lidar_selection', 'safety')}
    poses, ages, rows, first, last = [], [], 0, None, None
    with a.source.open() as stream:
        for line in stream:
            d = json.loads(line)
            if not a.start_mono <= d['mono'] <= a.end_mono:
                continue
            rows += 1
            first = first or d
            last = d
            s = d.get('snapshot') or {}
            h = s.get('health') or {}
            solution = d.get('solution') or {}
            counters['solution_mode'][solution.get('mode', 'MISSING')] += 1
            counters['solution_reason'][solution.get('reason', '')] += 1
            counters['map_state'][h.get('state', 'MISSING')] += 1
            counters['lidar_selection'][','.join(h.get('healthy_lidars', []))] += 1
            counters['safety'][d.get('safety', {}).get('reason', '')] += 1
            pose = solution.get('pose') or s.get('pose')
            if pose and len(pose) == 4 and all(math.isfinite(v) for v in pose):
                poses.append(pose)
            if isinstance(solution.get('map_age_s'), (float, int)):
                ages.append(solution['map_age_s'])
    def quantile(values, q):
        return sorted(values)[int((len(values)-1)*q)] if values else None
    summary = dict(scope='stationary pose repeatability, not absolute accuracy or navigation acceptance',
                   source=str(a.source), requested_mono=[a.start_mono, a.end_mono], samples=rows,
                   actual_mono=[first['mono'], last['mono']] if first else [],
                   counts={k: dict(v) for k,v in counters.items()},
                   map_age_s=dict(p50=quantile(ages,.5), p95=quantile(ages,.95), max=max(ages) if ages else None))
    if poses:
        summary['pose'] = dict(median=[statistics.median(v) for v in zip(*poses)],
                               min=[min(v) for v in zip(*poses)], max=[max(v) for v in zip(*poses)],
                               stdev=[statistics.pstdev(v) for v in zip(*poses)])
    if first:
        summary['ingest_delta'] = {k:last.get('ingest_counts',{}).get(k,0)-v for k,v in first.get('ingest_counts',{}).items()}
        summary['queue_drop_delta'] = last.get('queue_drops',0)-first.get('queue_drops',0)
    a.output.parent.mkdir(exist_ok=True)
    a.output.write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
