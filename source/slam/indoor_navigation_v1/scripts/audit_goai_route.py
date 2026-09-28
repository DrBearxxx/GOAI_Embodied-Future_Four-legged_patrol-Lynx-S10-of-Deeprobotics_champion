"""Summarize a read-only route log; never upgrades offline data to live approval."""
import argparse,collections,json,math
from pathlib import Path


def audit(rows):
    rows=[r for r in rows if 'command' in r and 'mono' in r]
    if not rows:return dict(ready_for_supervised_trial=False,reason='NO_SAMPLES')
    duration=rows[-1]['mono']-rows[0]['mono'];loc_valid=[];good=[];gap=0.;max_gap=0.;seen_valid=False;max_log_gap=0.
    v2_modes=collections.Counter();v2_available=[]
    for i,r in enumerate(rows):
        t=r['mono'];s=r.get('localization')
        try:
            h=s['health'];age=h['age_s']+max(0.,t-s.get('matching_snapshot_mono',s['mono']))
            valid=(0<=t-s['mono']<=.25 and 0<=age<=.35 and h['valid'] is True and h['single_lidar'] is False
                and set(h['healthy_lidars'])=={'front','rear'} and all(0<=t-s['perception_mono'][x]<=.25 for x in ('front','rear'))
                and s['front_fresh'] is True and s['rear_fresh'] is True and not s.get('error'))
        except (KeyError,TypeError,ValueError):valid=False
        if s and s.get('schema')=='goai.localization.continuity.v2':
            try:
                sol=s['solution'];mode=sol['mode'];v2_modes[mode]+=1
                available=(mode in ('TRACKING','DEGRADED','PREDICT_ONLY') and 0<=t-s['mono']<=.25 and
                    0<=t-sol['measurement_mono']<=.55 and 0<=t-sol['estimate_mono']<=.08 and not s.get('error'))
            except (KeyError,TypeError,ValueError):available=False
            v2_available.append(available)
        loc_valid.append(valid);good.append(r['command'].get('ready') is True)
        dt=0 if i==0 else max(0.,t-rows[i-1]['mono'])
        max_log_gap=max(max_log_gap,dt)
        # Report gaps after the first valid observation, not initialization.
        seen_valid |= valid
        if seen_valid:
            gap=0. if valid else gap+dt;max_gap=max(max_gap,gap)
    valid_fraction=sum(loc_valid)/len(rows)
    return dict(seconds=duration,samples=len(rows),localization_valid_fraction=valid_fraction,
        v2_modes=dict(v2_modes),v2_samples=len(v2_available),
        v2_estimate_available_fraction=sum(v2_available)/len(rows) if v2_available else None,
        v2_conditional_available_fraction=sum(v2_available)/len(v2_available) if v2_available else None,
        v2_note='Continuity availability is separate from obstacle clearance, route corridor and GOAI authorization; legacy timing metric retained.',
        longest_invalid_after_first_valid_s=max_gap,max_log_gap_s=max_log_gap,gate_ready_fraction=sum(good)/len(rows),
        states=dict(collections.Counter(r['command']['state'] for r in rows)),
        execute=any(r['execute'] for r in rows),sent=rows[-1]['sent'],nonzero_sent=rows[-1]['nonzero_sent'],
        ready_for_supervised_trial=bool(duration>=30 and valid_fraction>=.95 and max_gap<=.35 and max_log_gap<=.2 and all(good[-50:]) and len(good)>=50),
        note='Readiness screen only; still requires onsite pose agreement, clear ground, G12/E-stop, ROUTE-READY + NAV-READY + C. No absolute accuracy claim.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('log',nargs='?',help='Default: latest GOAI route log');a=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    path=Path(a.log) if a.log else max((root/'live_logs').glob('goai-route-*.jsonl'),key=lambda x:x.stat().st_mtime)
    rows=[]
    for line in path.read_text().splitlines():
        try:rows.append(json.loads(line))
        except json.JSONDecodeError:pass
    print(json.dumps(dict(log=str(path),**audit(rows)),ensure_ascii=False,indent=2))
