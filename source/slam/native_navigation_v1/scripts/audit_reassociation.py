"""Audit recorded corrections, not simulated permission or physical accuracy."""
import argparse
from collections import Counter
import gzip
import json
import math
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('health');p.add_argument('--output',required=True)
    p.add_argument('--warmup',type=float,default=30);p.add_argument('--duration',type=float,default=1200)
    a=p.parse_args();counts=Counter();first=None;prior=None;last=None;max_step=0.;velocities=[];measured_steps=[]
    opener=gzip.open if a.health.endswith('.gz') else open
    with opener(a.health,'rt') as stream:
        for line in stream:
            row=json.loads(line);now=row['mono'];first=now if first is None else first
            if not a.warmup<=now-first<a.warmup+a.duration:continue
            s=row.get('solution')
            if not s:continue
            counts['samples']+=1;counts['mode_'+s['mode']]+=1;last=now
            velocities.append(math.sqrt(sum(v*v for v in s['velocity_body'])))
            if prior:
                t,old=prior;dt=min(.55,now-t);p0,p1=old['pose'],s['pose']
                distance=math.dist(p0[:3],p1[:3]);angle=abs(math.atan2(math.sin(p1[3]-p0[3]),math.cos(p1[3]-p0[3])))
                jump=distance>.15+.35*dt or angle>.20+.4*dt;max_step=max(max_step,distance)
                if jump:counts['actual_pose_step_over_short_budget']+=1
                if s['measurement_mono']!=old['measurement_mono']:
                    measured_steps.append(math.dist(s['measured_pose'][:3],old['measured_pose'][:3]))
                if old['generation']!=s['generation']:
                    counts['generation_transitions']+=1
                    if old['generation'][0]!=s['generation'][0]:counts['map_epoch_transitions']+=1
                    else:
                        counts['continuity_counter_transitions']+=1
                        counts['counter_transition_with_pose_jump' if jump else 'counter_transition_without_pose_jump']+=1
            prior=now,s
    def quantiles(v):
        v=sorted(v);return {str(q):v[min(len(v)-1,int(q*(len(v)-1)))] for q in (.5,.9,.95,.99,1)} if v else {}
    result=dict(scope='recorded geometry/counter audit only; not route permission, raw VIO replay or motion validation',
                source=str(a.health),warmup_s=a.warmup,duration_s=a.duration,counts=dict(counts),
                max_successive_pose_step_m=max_step,velocity_quantiles_mps=quantiles(velocities),measured_step_quantiles_m=quantiles(measured_steps),physical_motion=False,
                limitation='No raw IMU/VIO history here; cannot claim the new odometry bridge would have recovered recorded losses. Safety/authority/corridor still gate movement.')
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
