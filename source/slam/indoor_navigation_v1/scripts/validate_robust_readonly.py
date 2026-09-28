"""Bounded live read-only check. Does not launch GOAI, authority or sensors."""
import collections,datetime,json,signal,subprocess,time
from pathlib import Path
from audit_goai_route import audit

root=Path(__file__).resolve().parents[1]
out=root/'results'/('robust-readonly-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir(parents=True,exist_ok=False);logs=root/'live_logs';before=set(logs.glob('*.jsonl'))
files=[];processes=[];start=time.monotonic();last=start;route_started=False
try:
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:args=[Path(v.decode()).name for v in p.read_bytes().split(b'\0') if v]
        except (OSError,UnicodeDecodeError):continue
        assert not set(args)&{'goai_node','goai_route.py','localize_mp.py','localize.py'},'Conflicting running process'
    f=(out/'localizer.stdout').open('x');files.append(f)
    processes.append(subprocess.Popen(['bash','run_goai_localizer.sh','--seconds','72'],cwd=root,stdout=f,stderr=subprocess.STDOUT))
    while time.monotonic()-start<90:
        now=time.monotonic()
        if not route_started and now-start>=8:
            f=(out/'route.stdout').open('x');files.append(f)
            processes.append(subprocess.Popen(['bash','run_goai_route.sh','--seconds','60'],cwd=root,stdout=f,stderr=subprocess.STDOUT));route_started=True
        if now-last>=10:print(json.dumps(dict(seconds=round(now-start,1),exit_codes=[p.poll() for p in processes],mode='READ_ONLY')),flush=True);last=now
        if route_started and all(p.poll() is not None for p in processes):break
        time.sleep(.1)
finally:
    for p in processes:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
    for f in files:f.close()
new=sorted(set(logs.glob('*.jsonl'))-before)
def rows(p):
    with p.open() as f:
        for line in f:
            try:yield json.loads(line)
            except json.JSONDecodeError:continue
report=dict(out=str(out),exit_codes=[p.returncode for p in processes],physical_motion=False,new_logs=[str(p) for p in new])
for p in new:
    if p.name.startswith('goai-route-'):
        report['route_audit']=audit(list(rows(p)))
    if p.name.startswith('health-'):
        health=list(rows(p));tail=[r for r in health if r['mono']-health[0]['mono']>=8];n=len(tail)
        modes=collections.Counter((r.get('solution') or {}).get('mode','NONE') for r in tail)
        reasons=collections.Counter((r.get('solution') or {}).get('reason','') for r in tail)
        clear_modes=collections.Counter((r.get('safety') or {}).get('mode','NONE') for r in tail)
        data_fresh=sum(all(s in r.get('safety_records',{}) and 0<=r['mono']-r['safety_records'][s]['mono']<=.25
            for s in ('front','rear')) for r in tail)
        report['live_estimator']=dict(samples=n,modes=dict(modes),reasons=dict(reasons),safety_modes=dict(clear_modes),
            dual_safety_data_fresh_fraction=data_fresh/max(n,1),estimate_available_fraction=1-(modes['LOST']+modes['NONE'])/max(n,1),
            last_counts=health[-1]['ingest_counts'],last_error=health[-1]['error'],last_input_rejects=health[-1].get('input_rejects'))
(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
assert all(p.returncode==0 for p in processes),'Read-only child failed; inspect stdout'
assert report.get('route_audit',{}).get('sent')==0,'Unexpected command publisher'
