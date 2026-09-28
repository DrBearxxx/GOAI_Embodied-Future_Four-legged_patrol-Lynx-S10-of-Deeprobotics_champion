"""Apply only reviewed navigation sources; preserve calibration, progress and LIO."""
from pathlib import Path
import argparse,hashlib,json,os,shlex,shutil,signal,subprocess,sys,tempfile,time
APP=Path('/home/wym/s10_navigation_app_v1');PACKAGE=Path(__file__).resolve().parent
VERSION='1.33-auto-map'
def digest(p):return hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n')).hexdigest()
def atomic_json(p,v):
    temp=p.with_name(p.name+'.auto-map-tmp');temp.write_text(json.dumps(v,ensure_ascii=False));os.replace(temp,p)
def pid():
    p=subprocess.run(['tmux','display-message','-p','-t','goai-navigation:0','#{pane_pid}'],capture_output=True,text=True,check=True)
    n=int(p.stdout.strip())
    if Path(f'/proc/{n}/cwd').resolve()!=APP or b'runtime.py' not in Path(f'/proc/{n}/cmdline').read_bytes():raise RuntimeError('Unexpected navigation process')
    return n
def stop(n):
    os.kill(n,signal.SIGINT)
    end=time.monotonic()+20
    while Path(f'/proc/{n}').exists() and time.monotonic()<end:time.sleep(.1)
    if Path(f'/proc/{n}').exists():raise RuntimeError('Navigation process did not exit')
def start():
    subprocess.run(['tmux','new-session','-d','-s','goai-navigation',
        'cd /home/wym/s10_navigation_app_v1 && exec bash run.sh --port 18894 --gateway-url http://127.0.0.1:18895 >> logs/runtime.log 2>&1'],check=True)
def check(manifest):
    for e in manifest:
        if '..' in Path(e['path']).parts or Path(e['path']).is_absolute():raise ValueError('Invalid manifest path')
        if digest(PACKAGE/'app'/e['path'])!=e['sha256']:raise RuntimeError('Package changed: '+e['path'])
        installed=APP/e['path'];before=digest(installed) if installed.exists() else None
        if before not in (e['before'],e['sha256']):raise RuntimeError('Installed source changed: '+e['path'])
def checkpoint(s):
    n=s['navigation'];m=s['mission']
    return dict(asset_id=s['asset_id'],route_id=n['route_id'],route=m['current_route'],target_index=n['target_index'],
        reached_indices=n['reached_indices'],skipped_indices=n.get('skipped_indices',[]),override=m['override'],manual_policy=m['manual_policy'],
        input_kind=n['input_kind'],manual_target=n.get('manual_target'),manual_progress=n.get('manual_progress'),temporary_waypoints=n.get('temporary_waypoints'))
def idle(call,query):
    s=call('/state',{'include_catalog':True});c=query();n=s['navigation']
    if n['owner']!='paused' or n['active'] or c['enabled'] or any(c['output']):raise RuntimeError('Robot is being controlled; leave the running process untouched')
    return s
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--deploy',action='store_true');args=parser.parse_args()
    manifest=json.loads((PACKAGE/'manifest.json').read_text());check(manifest)
    evidence=PACKAGE/'validation.json'
    if not args.deploy:
        review=Path(tempfile.mkdtemp(prefix='review-',dir=PACKAGE))
        shutil.copytree(APP,review/'app',ignore=shutil.ignore_patterns('logs','private','__pycache__','.git','*.apk','build','.gradle'))
        for e in manifest:
            target=review/'app'/e['path'];target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(PACKAGE/'app'/e['path'],target)
        script='''#!/usr/bin/env bash
set -eo pipefail
source /home/wym/s10_slam/env.sh
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1
cd REVIEW
python3 -m unittest discover -v
'''.replace('REVIEW',shlex.quote(str(review/'app')))
        (review/'tests.sh').write_text(script)
        with (PACKAGE/'validation.log').open('w') as log:subprocess.run(['bash',str(review/'tests.sh')],stdout=log,stderr=subprocess.STDOUT,check=True)
        atomic_json(evidence,dict(manifest=manifest,passed=True,review=str(review)))
        print('Isolated Orin navigation tests passed. Running service unchanged.');return
    validated=json.loads(evidence.read_text())
    if not validated['passed'] or validated['manifest']!=manifest:raise RuntimeError('Validate this exact package first')
    sys.path.insert(0,str(APP));from startup_status import call;from control_status import query
    state=idle(call,query);saved=checkpoint(state)
    private={p.name:digest(p) for p in (APP/'private').glob('*.json') if not p.name.startswith('navigation-resume')}
    backup=Path(tempfile.mkdtemp(prefix='pre-auto-map-',dir=APP/'logs'))
    previous=[]
    for e in manifest:
        p=APP/e['path'];previous.append((e['path'],p.exists()))
        if p.exists():dest=backup/e['path'];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest)
    atomic_json(backup/'progress.json',saved)
    atomic_json(backup/'position.json',dict(pose=state.get('pose'),map_id=state['map_id']))
    idle(call,query);process=pid();stop(process)
    try:
        for e in manifest:
            p=APP/e['path'];p.parent.mkdir(parents=True,exist_ok=True)
            temp=p.with_name(p.name+'.auto-map-tmp');shutil.copyfile(PACKAGE/'app'/e['path'],temp);os.replace(temp,p)
        atomic_json(APP/'private/navigation-resume.json',saved);start()
        deadline=time.monotonic()+45
        while True:
            try:
                after=call('/state',{'include_catalog':True})
                if after.get('navigation_version')!=VERSION:raise RuntimeError('Wrong running version')
                break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.25)
        n=after['navigation'];control=query()
        assert n['owner']=='paused' and not n['active'] and not control['enabled'] and not any(control['output'])
        for name in ('route_id','target_index','reached_indices','skipped_indices','temporary_waypoints'):assert n.get(name)==saved.get(name),name
        assert after['mission']['current_route']==saved['route']
        for name,sha in private.items():assert digest(APP/'private'/name)==sha,name
    except Exception:
        try:stop(pid())
        except (ProcessLookupError,subprocess.CalledProcessError):pass
        for name,existed in previous:
            p=APP/name
            if existed:shutil.copyfile(backup/name,p)
            elif p.exists():p.unlink()
        atomic_json(APP/'private/navigation-resume.json',saved);start();raise
    result=dict(version=VERSION,backup=str(backup),route=saved['route_id'],target=saved['target_index'],
                calibration_revision=after['mission']['calibration_revision'],progress_preserved=True,
                lightning_restarted=False,gateway_restarted=False,mode=after['mode'],last_pose=state.get('pose'))
    atomic_json(PACKAGE/'deployment.json',result);print(json.dumps(result,ensure_ascii=False))
if __name__=='__main__':main()
