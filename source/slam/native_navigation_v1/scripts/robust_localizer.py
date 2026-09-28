"""V3 read-only localization runtime. Never creates robot command publishers."""
import argparse
import fcntl
from functools import partial
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_nav.bootstrap import ROOT, INDOOR, verify
verify()
sys.path.insert(0,str(INDOOR/'scripts'))
import localize_mp
from native_nav.robust_timing import RobustContinuity, use_slewed_clocks
from native_nav.robust_worker import robust_worker_main
from native_nav.bounded_log import BoundedLog
from native_nav.watchdog import HeartbeatWatchdog
import rclpy


class RestartLocalizer(RuntimeError):
    pass


class NoPublisher:
    def publish(self,value):pass


class RobustLocalizer(localize_mp.Localizer):
    def __init__(self,args):
        # Explicit dependency selection for the inherited ingress; frozen files
        # remain untouched. The spawned worker imports its own V3 clock factory.
        localize_mp.use_live_clocks=use_slewed_clocks
        self.worker_heartbeat=mp.get_context('spawn').Value('d',0.,lock=False)
        localize_mp.worker_main=partial(robust_worker_main,heartbeat=self.worker_heartbeat)
        self.worker_watchdog=HeartbeatWatchdog(time.monotonic())
        self.last_heartbeat=0.
        super().__init__(args)
        self.log.close()
        self.log=BoundedLog(ROOT/'live_logs'/'robust-health.jsonl')
        self.continuity=RobustContinuity()
        if args.no_publish:
            self.destroy_publisher(self.pub)
            self.pub=NoPublisher()

    def input(self,topic,msg,info):
        try:
            return super().input(topic,msg,info)
        except (AttributeError,ValueError,TypeError,KeyError,IndexError,OverflowError) as exc:
            old=self.input_rejects.get(topic,{})
            self.input_rejects[topic]=dict(count=old.get('count',0)+1,last_mono=time.monotonic(),reason=repr(exc))
            # No pose re-stamping and no freshness renewal on failed input.

    def camera_info(self,msg):
        try:
            return super().camera_info(msg)
        except (AttributeError,ValueError,TypeError,IndexError,OverflowError):
            return

    def publish(self):
        super().publish()
        now=time.monotonic()
        if self.worker_heartbeat.value>0:self.worker_watchdog.beat(self.worker_heartbeat.value)
        if self.worker_watchdog.expired(now):raise RestartLocalizer('matching_worker_unresponsive')
        if self.error in ('matching_worker_exited','safety_worker_exited') or (self.error and not self.worker.is_alive()):
            raise RestartLocalizer(self.error)
        if now-self.last_heartbeat>=1.:
            print(json.dumps(dict(localizer_heartbeat=True,run_id=str(self.run_id))),flush=True)
            self.last_heartbeat=now

    def close(self):
        try:super().close()
        finally:
            # SIGTERM alone cannot reap a SIGSTOP'ed child. Kill only our own
            # multiprocessing children if the normal bounded shutdown failed.
            for process in (self.worker,self.safety_process):
                if process.is_alive():process.kill();process.join(timeout=2)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=0)
    p.add_argument('--initial',nargs=4,type=float)
    p.add_argument('--no-publish',action='store_true')
    p.add_argument('--profile',action='store_true')
    a=p.parse_args()
    if a.seconds<0:p.error('seconds must be >=0')
    lock=open('/tmp/wym-indoor-localizer.lock','a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:return 73
    rclpy.init()
    n=None
    code=0
    try:
        n=RobustLocalizer(a)
        start=time.monotonic()
        print(json.dumps(dict(implementation='native_v3',run_id=str(n.run_id),pid=__import__('os').getpid(),
                             worker_pid=n.worker.pid,no_publish=a.no_publish,motion_commands_sent=0)),flush=True)
        while rclpy.ok() and (not a.seconds or time.monotonic()-start<a.seconds):
            rclpy.spin_once(n,timeout_sec=.01)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print('LOCALIZER_RESTART_REQUIRED: '+repr(exc),file=sys.stderr,flush=True)
        code=72
    finally:
        if n:n.close()
        if rclpy.ok():rclpy.shutdown()
        lock.close()
    return code


if __name__=='__main__':
    sys.exit(main())
