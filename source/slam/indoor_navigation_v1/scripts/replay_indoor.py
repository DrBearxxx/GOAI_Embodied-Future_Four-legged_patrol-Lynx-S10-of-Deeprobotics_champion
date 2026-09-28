"""Causal localization on the new crop; no reference trajectory at runtime."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT,BASE
from indoor.clock import use_live_clocks
from s10nav.util import read,write,sha,stats
from s10nav.engine import Engine
from s10nav.matching import Matcher
from s10nav.replay_io import Replay,inject,SCENARIOS

FAULTS=[dict(start=5,end=7,sensor='front',action='drop'),dict(start=10,end=12,sensor='rear',action='stamp_shift',seconds=1.),
    dict(start=15,end=17,sensor='all',action='drop'),dict(start=20,end=22,sensor='camera',kind='vio',action='vio_jump',metres=15.),
    dict(start=24,end=26,sensor='front',action='delay',seconds=.6)]
def main():
    p=argparse.ArgumentParser();p.add_argument('--session',choices=['A','B'],default='A');p.add_argument('--faults',action='store_true')
    p.add_argument('--duration',type=float,default=35);p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False);cfg=read(ROOT/'config.json');assets=ROOT/'assets'
    for f,h in read(assets/'manifest.json')['files'].items():assert sha(assets/f)==h
    # Blind initialization within the explicitly restricted indoor gallery.
    engine=use_live_clocks(Engine(assets,cfg,Matcher(assets,cfg)));SCENARIOS['indoor_faults']=FAULTS
    scenario='indoor_faults' if a.faults else 'baseline';tick=0.;rows=[];health=[];cost=[];start=time.perf_counter();tail=0.;last=start
    with (out/'events.jsonl').open('x') as f,(out/'health.jsonl').open('x') as hf:
        for e in inject(Replay(BASE/'replay_data'/a.session).events(),scenario):
            now=e['received']
            if now>a.duration:break
            while tick<=now:
                h=engine.proposal(tick);hf.write(json.dumps(dict(t=tick,**h))+'\n');health.append(h['localization']['valid']);tick+=.1
            t=time.perf_counter();r=engine.event(e,now);tail=now
            if r:
                cost.append(time.perf_counter()-t);f.write(json.dumps(r)+'\n')
                if r['accepted']:
                    T=np.array(r['pose']);rows.append([r['t'],*T[:3,3],*Rotation.from_matrix(T[:3,:3]).as_quat(),int(r['health']['valid'])])
            if time.perf_counter()-last>15:print(a.session,scenario,round(now,1),engine.state,engine.reason,flush=True);last=time.perf_counter()
        live_counts=dict(engine.counts)
        for now in np.arange(tail+.1,tail+1.1,.1):
            now=float(now);hf.write(json.dumps(dict(t=now,**engine.proposal(now)))+'\n')
    np.savetxt(out/'trajectory.csv',np.array(rows).reshape(-1,9),delimiter=',',header='t,x,y,z,qx,qy,qz,qw,valid',comments='')
    report=dict(session=a.session,scenario=scenario,duration_s=tail,valid_fraction=sum(health)/len(health),fault_rules=SCENARIOS[scenario],
        counts=live_counts,processing_seconds=stats(cost),wall_seconds=time.perf_counter()-start,initial_pose=None,
        query_reference_trajectory_used=False,robot_commands_sent=False,independent_accuracy_verified=False,
        route_control_evaluated=False,timing_scope='unpaced local replay; separate live timing acceptance required')
    write(out/'summary.json',report);print(json.dumps(report),flush=True)
    sys.path.insert(0,str(BASE/'scripts'));from assess_results import assess
    assess(out)
if __name__=='__main__':main()
