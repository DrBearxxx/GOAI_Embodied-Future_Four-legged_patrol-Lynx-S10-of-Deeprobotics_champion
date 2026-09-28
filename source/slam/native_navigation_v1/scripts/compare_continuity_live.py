"""Paired SAME-INPUT read-only comparison. Never publishes localization/control."""
import argparse
import fcntl
import hashlib
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import signal
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_nav.bootstrap import ROOT
from native_nav.robust_timing import RobustContinuity
from native_nav.bounded_log import BoundedLog
from robust_localizer import RobustLocalizer
import rclpy
from rclpy.signals import SignalHandlerOptions


class Comparison:
    def __init__(self,old,start,log,gaps=False):
        self.old=old;self.new=RobustContinuity();self.start=start;self.log=log
        self.counts={name:Counter() for name in ('old','new')};self.previous={};self.last_emit=start
        self.schedule=[(35.,.8),(60.,1.6)] if gaps else [];self.phase_counts={}
    def update(self,anchor,motion):
        elapsed=time.monotonic()-self.start
        if any(t<=elapsed<t+duration for t,duration in self.schedule):return
        self.old.update(anchor,motion);self.new.update(anchor,motion)
    def estimate(self,now,motion):
        results=dict(old=self.old.estimate(now,motion),new=self.new.estimate(now,motion))
        if now-self.start>=20:
            for name,s in results.items():
                if not s:continue
                c=self.counts[name];c['samples']+=1;c['mode_'+s['mode']]+=1
                for start,duration in self.schedule:
                    if start<=now-self.start<start+duration:
                        key=str(duration)+'s_map_input_pause_'+name
                        phase=self.phase_counts.setdefault(key,Counter());phase[s['mode']]+=1
                if s.get('fusion_method'):c[s['fusion_method']]+=1
                if s.get('velocity_source'):c[s['velocity_source']]+=1
                if s['mode']=='LOST':c['lost_'+s['reason']]+=1
                if s.get('bridge_reject'):c['bridge_reject_'+s['bridge_reject']]+=1
                if name in self.previous:
                    t,prior=self.previous[name];dt=min(.55,now-t)
                    distance=math.dist(s['pose'][:3],prior['pose'][:3])
                    angle=abs(math.atan2(math.sin(s['pose'][3]-prior['pose'][3]),math.cos(s['pose'][3]-prior['pose'][3])))
                    if distance>.15+.35*dt or angle>.20+.4*dt:c['pose_step_over_short_budget']+=1
                self.previous[name]=now,s
            self.log.write(json.dumps(dict(mono=now,**results))+'\n')
        if now-self.last_emit>=10:
            print(json.dumps(dict(readonly=True,elapsed_s=round(now-self.start,1),counts={k:dict(v) for k,v in self.counts.items()})),flush=True)
            self.last_emit=now
        return results['new']


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=float,default=140)
    p.add_argument('--baseline',required=True);p.add_argument('--inject-map-gaps',action='store_true');a=p.parse_args()
    lock=open('/tmp/wym-indoor-localizer.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    spec=importlib.util.spec_from_file_location('baseline_robust_timing',a.baseline)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    baseline_hash=hashlib.sha256(Path(a.baseline).read_bytes()).hexdigest()
    root=ROOT/'results'/('paired-readonly-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir(parents=True)
    log=BoundedLog(root/'paired.jsonl',16*1024*1024,1)
    stop=[False]
    def end(*unused):stop[0]=True
    for s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,end)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=None;started=time.monotonic();code=0
    comparison=Comparison(module.RobustContinuity(),started,log,a.inject_map_gaps)
    try:
        node=RobustLocalizer(argparse.Namespace(initial=None,profile=False,no_publish=True))
        node.continuity=comparison
        while not stop[0] and time.monotonic()-started<a.seconds:rclpy.spin_once(node,timeout_sec=.01)
    except Exception as exc:
        code=1;print(repr(exc),file=sys.stderr,flush=True)
    finally:
        if node:node.close()
        if rclpy.ok():rclpy.shutdown()
        log.close()
        report=dict(scope='same-input real sensor continuity comparison; no navigation permission or physical motion',
                    seconds=time.monotonic()-started,warmup_s=20,exit_code=code,counts={k:dict(v) for k,v in comparison.counts.items()},
                    interrupted=stop[0],motion_commands_sent=0,localization_published=False,baseline=a.baseline,baseline_sha256=baseline_hash)
        report['software_injected_map_gaps']=comparison.schedule
        report['injected_gap_counts']={k:dict(v) for k,v in comparison.phase_counts.items()}
        (root/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(str(root),flush=True)
    return code


if __name__=='__main__':sys.exit(main())
