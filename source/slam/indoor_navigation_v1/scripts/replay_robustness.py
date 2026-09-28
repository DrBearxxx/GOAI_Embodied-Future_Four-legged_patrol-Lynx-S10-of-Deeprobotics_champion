"""Causal real-sensor replay. No reference trajectory or actuator publishers."""
import argparse,json,sys,time
from collections import Counter
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT,BASE
from indoor.clock import use_live_clocks
from indoor.pipeline import IndoorMatcher,diagnostic
from indoor.continuity import Continuity,use_redundancy,safety_policy
from indoor.safety_stream import clearance
from s10nav.engine import Engine
from s10nav.replay_io import Replay,inject,SCENARIOS
from s10nav.util import read,sha,apply

RULES=[dict(start=0,end=40,sensor='camera',action='drop'),
    dict(start=8,end=8.25,sensor='front',kind='imu',action='drop'),
    dict(start=22,end=22.25,sensor='rear',kind='cloud',action='drop'),
    dict(start=25,end=25.2,sensor='all',kind='imu',action='drop'),
    dict(start=27,end=27.5,sensor='front',action='stamp_shift',seconds=1.),
    dict(start=28,end=28.4,sensor='rear',action='delay',seconds=.6),
    dict(start=29,end=29.4,sensor='front',action='duplicate'),
    dict(start=30,end=31,sensor='all',action='drop')]

def main():
    p=argparse.ArgumentParser();p.add_argument('--faults',action='store_true');p.add_argument('--duration',type=float,default=35)
    p.add_argument('--session',choices=['A','B'],default='A');p.add_argument('--output',required=True);args=p.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False);assets=ROOT/'assets';cfg=read(ROOT/'config.json')
    for f,h in read(assets/'manifest.json')['files'].items():assert sha(assets/f)==h
    e=use_redundancy(use_live_clocks(Engine(assets,cfg,IndoorMatcher(assets,cfg))));c=Continuity()
    safety={};keys={};tick=0.;seq=0;counts=Counter();reasons=Counter();modes=Counter();legacy=0;costs=[];n=0;warm=0
    begin=time.perf_counter();last=begin;last_t=0.;sensor_counts=Counter();SCENARIOS['robust_v2']=RULES
    scenario='robust_v2' if args.faults else 'baseline';stalls=[(12.,12.25),(18.,18.4)] if args.faults else []
    with (out/'health.jsonl').open('x') as f,(out/'matches.jsonl').open('x') as mf:
        def evaluate(t):
            nonlocal legacy,n,warm
            solution=c.estimate(t,e.motion)
            for side in list(safety):
                if safety[side]['epoch']!=e.clocks[side].epoch:del safety[side]
            coverage=safety_policy(t,safety);h=e.poll(t)
            # Comparable old timing/dual criteria; obstacle content is separate.
            old=bool(h['valid'] and not h['single_lidar'] and h['age_s'] is not None and h['age_s']<=.35 and
                all(s in safety and 0<=t-safety[s]['mono']<=.25 for s in ('front','rear')))
            if t>=5:
                n+=1;legacy+=old;mode=solution['mode'] if solution else 'UNINITIALIZED';modes[mode]+=1
                why=solution.get('reason','') if solution else 'UNINITIALIZED'
                if why:reasons[why]+=1
            else:warm+=1
            f.write(json.dumps(dict(t=t,solution=solution,safety=coverage,safety_records=safety,legacy_timing_valid=old))+'\n')
        for event in inject(Replay(BASE/'replay_data'/args.session).events(),scenario):
            now=event['received']
            if now>args.duration:break
            while tick<now:evaluate(tick);tick+=.05
            last_t=now;sensor_counts[event['sensor']+'/'+event['kind']]+=1
            if event['kind']=='cloud':
                side=event['sensor'];clock=e.clocks[side];start=clock.map_stamp(event['stamp'],now)
                span=float(np.max(event['rel'])) if len(event['rel']) else 0.;key=(clock.epoch,event['stamp'])
                if (start is not None and key>keys.get(side,(-1,-1e30)) and .03<span<.16 and
                        0<=now-start-span<=.25 and len(event['points'])>=300):
                    keys[side]=key;safety[side]=clearance(apply(np.array(e.ext['lidar'][side]),event['points']),start+span,clock.epoch)
                if any(a<=now<b for a,b in stalls):counts['matching_scan_skipped_by_stall']+=1;continue
            t=time.perf_counter();r=e.event(event,now)
            if event['kind']=='cloud':costs.append(time.perf_counter()-t)
            if r:
                mf.write(json.dumps(r)+'\n');counts['accepted' if r['accepted'] else 'rejected']+=1
                if r['accepted']:
                    seq+=1;s=diagnostic(e,now,seq,{},{});c.update(s['anchor'],e.motion)
            if time.perf_counter()-last>20:
                print(args.session,scenario,round(now,2),e.state,dict(modes),flush=True);last=time.perf_counter()
        while tick<=args.duration+1:evaluate(tick);tick+=.05
    summary=dict(session=args.session,scenario=scenario,recorded_duration_s=last_t,analysis_samples_after_5s_including_1s_tail=n,
        sensor_counts=dict(sensor_counts),modes=dict(modes),reasons=dict(reasons),match_counts=dict(counts),
        legacy_timing_valid_fraction=legacy/max(n,1),continuity_available_fraction=1-(modes['LOST']+modes['UNINITIALIZED'])/max(n,1),
        fault_rules=RULES if args.faults else [],matching_stalls=stalls,wall_seconds=time.perf_counter()-begin,
        matching_ms_p95=float(np.quantile(costs,.95)*1000) if costs else None,
        query_reference_trajectory_used=False,physical_motion=False,absolute_accuracy_verified=False,
        limitations=['Unpaced replay does not prove on-device scheduling latency.',
            'Recorded clouds are reduced; replay obstacle results are not live collision acceptance.',
            'Availability is not route authorization or localization accuracy.',
            'Engineering prediction budgets are not calibrated confidence bounds.'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)

if __name__=='__main__':main()
