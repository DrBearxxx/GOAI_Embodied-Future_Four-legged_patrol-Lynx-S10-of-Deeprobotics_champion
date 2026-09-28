"""Restart only the read-only localizer. Never restart or arm locomotion/routes."""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_nav.bootstrap import ROOT, verify
from native_nav.bounded_log import BoundedLog
from native_nav.watchdog import HeartbeatWatchdog


def stop_group(child,grace=8):
    try:os.killpg(child.pid,signal.SIGINT)
    except ProcessLookupError:pass
    try:child.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        try:os.killpg(child.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        child.wait(timeout=2)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=0)
    p.add_argument('--no-publish',action='store_true')
    p.add_argument('--initial',nargs=4,type=float)
    a=p.parse_args()
    if a.seconds<0:p.error('seconds must be >=0')
    verify()  # A changed map/code baseline is fatal, not an automatic retry.
    stop=False
    def end(*unused):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,end)
    signal.signal(signal.SIGTERM,end)
    began=time.monotonic();attempt=0;launches=0;child=None
    log=BoundedLog(ROOT/'live_logs'/'robust-supervisor.jsonl',1024*1024,3)
    try:
        while not stop and (not a.seconds or time.monotonic()-began<a.seconds):
            verify()
            cmd=[sys.executable,str(ROOT/'scripts'/'robust_localizer.py')]
            if a.no_publish:cmd+=['--no-publish']
            if a.seconds:cmd+=['--seconds',str(max(.1,a.seconds-(time.monotonic()-began)))]
            # Explicit initial pose is valid only for first startup; a crashed
            # localizer must retrieve again, never reuse stale initial position.
            if a.initial and launches==0:cmd+=['--initial',*[str(v) for v in a.initial]]
            child=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True,bufsize=0)
            launches+=1
            child_started=time.monotonic()
            watchdog=HeartbeatWatchdog(child_started)
            unresponsive=False
            print(json.dumps(dict(supervisor='started',pid=child.pid,attempt=attempt,motion_commands_sent=0)),flush=True)
            pending=b''
            while child.poll() is None and not stop:
                if a.seconds and time.monotonic()-began>=a.seconds:
                    stop=True;break
                if watchdog.expired(time.monotonic()):
                    unresponsive=True
                    print(json.dumps(dict(supervisor='unresponsive',pid=child.pid)),flush=True)
                    stop_group(child,grace=2)
                    break
                ready,_,_=select.select([child.stdout],[],[],.2)
                if ready:
                    chunk=os.read(child.stdout.fileno(),8192)
                    pending+=chunk
                    if len(pending)>32768:pending=pending[-32768:]
                    while b'\n' in pending:
                        line,pending=pending.split(b'\n',1)
                        decoded=line.decode(errors='replace')
                        try:event=json.loads(decoded)
                        except json.JSONDecodeError:event=None
                        if isinstance(event,dict) and event.get('localizer_heartbeat') is True:
                            watchdog.beat(time.monotonic())
                            continue
                        log.write(json.dumps(dict(mono=time.monotonic(),pid=child.pid,line=decoded))+'\n')
                        if any(k in decoded for k in ('run_id','LOCALIZER_RESTART','LOG_UNAVAILABLE')):
                            print(decoded,flush=True)
            if stop:break
            code=child.wait();child.stdout.close()
            # A killed parent can leave multiprocessing workers behind. Reap
            # only the process group created by this supervisor's own Popen.
            try:os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            print(json.dumps(dict(supervisor='child_exit',code=code,pid=child.pid)),flush=True)
            if code==73 or code==0 and not unresponsive:break
            stable=time.monotonic()-child_started>=60
            attempt=0 if stable else attempt+1
            delay=min(30.,2.**min(attempt,5))
            print(json.dumps(dict(supervisor='retry_after',seconds=delay,requires_navigation_rearm=True)),flush=True)
            until=time.monotonic()+delay
            while not stop and time.monotonic()<until:
                time.sleep(.1)
    finally:
        if child and child.poll() is None:
            # Own process group only; its read-only children never own motors.
            stop_group(child)
        if child:
            try:os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        if child and child.stdout:child.stdout.close()
        log.close()


if __name__=='__main__':
    main()
