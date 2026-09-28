import argparse,json,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,write,sha,stats
from s10nav.matching import Matcher
from s10nav.engine import Engine
from s10nav.replay_io import Replay,inject,SCENARIOS

def run(session,scenario,duration,initial,out,assets=ROOT/'assets'):
    out.mkdir(parents=True,exist_ok=False);cfg=read(ROOT/'config.json');manifest=read(assets/'manifest.json')
    for name,expected in manifest['files'].items():
        if sha(assets/name)!=expected:raise ValueError('Asset checksum mismatch: '+name)
    matcher=Matcher(assets,cfg);T=None
    if initial is not None:
        T=np.eye(4);T[:3,3]=initial[:3];T[:3,:3]=Rotation.from_euler('z',np.deg2rad(initial[3])).as_matrix()
    engine=Engine(assets,cfg,matcher,T);begin=time.perf_counter();last=begin;tick=0.;n=0;times=[];rows=[];health_rows=[];nav_states={};tail=0
    with (out/'events.jsonl').open('w') as f,(out/'health.jsonl').open('w') as hf:
        for e in inject(Replay(ROOT/'replay_data'/session).events(),scenario):
            now=max(tail,e['received'])
            if duration and now>duration:break
            while tick<=now:
                proposal=engine.proposal(tick);health_rows.append((tick,proposal['localization']['valid'],proposal['localization']['state']))
                nav_states[proposal['state']]=nav_states.get(proposal['state'],0)+1
                hf.write(json.dumps(dict(t=tick,**proposal))+'\n');tick+=.1
            start=time.perf_counter();result=engine.event(e,now);dt=time.perf_counter()-start;tail=now
            if result is not None:
                n+=1;times.append(dt);f.write(json.dumps(result)+'\n')
                if result['accepted']:rows.append([result['t'],*np.array(result['pose'])[:3,3],*Rotation.from_matrix(np.array(result['pose'])[:3,:3]).as_quat(),int(result['health']['valid'])])
            if time.perf_counter()-last>15:
                print('REPLAY',session,scenario,round(now,1),'state',engine.state,'accepted',sum(engine.counts['accepted_'+s] for s in ['front','rear']),'reason',engine.reason,flush=True);last=time.perf_counter()
        live_counts=dict(engine.counts)
        # Explicit terminal outage: a replay ending must not leave a valid pose forever.
        for t in np.arange(tail+.1,tail+1.1,.1):
            proposal=engine.proposal(float(t));hf.write(json.dumps(dict(t=float(t),**proposal))+'\n')
    np.savetxt(out/'trajectory.csv',np.array(rows).reshape(-1,9),delimiter=',',header='t,x,y,z,qx,qy,qz,qw,valid',comments='')
    valid=sum(x[1] for x in health_rows);losses=live_counts.get('loss_transitions',0)
    report=dict(session=session,scenario=scenario,duration_s=tail,health_ticks=len(health_rows),valid_ticks=valid,
        valid_fraction=valid/max(len(health_rows),1),accepted_pose_count=len(rows),loss_transitions=losses,
        processing_seconds=stats(times),wall_seconds=time.perf_counter()-begin,counts=live_counts,
        navigation_state_ticks=nav_states,waypoints_reached=engine.route.reached,terminal_watchdog_valid=engine.poll(tail+1)['valid'],
        initial_pose=initial,fault_rules=SCENARIOS[scenario],causal_processing=True,query_reference_trajectory_used=False,
        online_clock_estimator='past IMU arrival/native pairs only; epoch quarantine on discontinuity',
        map_contains_query_session=True,independent_position_accuracy_verified=False,robot_commands_sent=False,
        timing_scope='unpaced offline processing; not end-to-end live latency or real-time certification')
    write(out/'summary.json',report);print(json.dumps(report),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--session',choices=['A','B'],required=True);p.add_argument('--scenario',choices=list(SCENARIOS),default='baseline')
    p.add_argument('--duration',type=float,default=0);p.add_argument('--initial',nargs=4,type=float);p.add_argument('--output',required=True);a=p.parse_args()
    run(a.session,a.scenario,a.duration,a.initial,Path(a.output))
if __name__=='__main__':main()
