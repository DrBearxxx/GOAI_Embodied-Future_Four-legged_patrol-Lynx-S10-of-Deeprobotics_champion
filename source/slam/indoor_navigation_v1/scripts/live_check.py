"""Bounded sensing/localization-only check, restores sensors it starts.

No SDK import, no robot-mode changes, no robot-command publisher.
"""
import json,os,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,'/home/wym/s10_slam/python')
import control
created=[];p=None
try:
    if any(control.alive(v) for k,v in control.state().items() if k in ['recorder','replayer','lio','localizer']):
        raise SystemExit('Refuse concurrent old recorder/replayer/localizer')
    for label in ['front','rear','insight9']:
        previous=control.state().get(label)
        if previous and control.alive(previous):continue
        args=[control.ROOT/'bin/airy_receiver','--ros-args','-p',f'side:={label}','-r',f'__node:=airy_{label}'] if label!='insight9' else [control.PY,control.APP/'insight9_bridge.py']
        _,entry=control.launch(label,args);created.append((label,entry))
    print('LIVE CHECK: sensors and localization only; no SDK process',flush=True)
    duration=float(os.environ.get('S10_INDOOR_CHECK_SECONDS','25'))
    args=['bash',str(ROOT/'run_localizer.sh'),'--seconds',str(duration)]
    if os.environ.get('S10_INDOOR_PROFILE')=='1':args+=['--profile']
    p=subprocess.Popen(args,cwd=ROOT)
    p.wait(timeout=duration+15)
    if p.returncode:raise RuntimeError('localizer exit '+str(p.returncode))
finally:
    if p is not None and p.poll() is None:p.terminate();p.wait(timeout=5)
    for label,entry in reversed(created):
        current=control.state().get(label)
        if current and current['tag']==entry['tag']:control.stop(label,timeout=8)
    print('LIVE CHECK FINISHED; only sensors created by this check were stopped',flush=True)
