"""Explicit single stand/hold/lie test. Default observes, never opens route control."""
import argparse
import json
from pathlib import Path
import signal
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native_nav.gate import Gate
from native_nav.gateway import Robot
from native_nav.posture import posture_packet, posture_reason


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--finish-standing', action='store_true', help='Observe existing regular-mode stand, then lie; never send STAND')
    p.add_argument('--hold-seconds', type=float, default=20)
    a = p.parse_args()
    if not 5 <= a.hold_seconds <= 30:
        p.error('hold must be 5..30 seconds')
    import fcntl
    lock = open('/tmp/s10-native-navigation-gateway.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    g = Gate()
    # OBSERVE Robot never emits axis or posture commands from tick/close.
    r = Robot(g, 'observe')
    out = Path(__file__).resolve().parents[1]/'results'
    out.mkdir(exist_ok=True)
    log = out/('posture-trial-'+str(time.time_ns())+'.jsonl')
    stopping = False
    phase = 'preflight'
    phase_at = time.monotonic()
    stable_at = None
    attempted_stand = False
    requested_lie = False
    printed = -1e9
    def stop(*unused):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(json.dumps(dict(log=str(log), execute=a.execute, axis_commands_enabled=False)), flush=True)
    with log.open('x') as stream:
        def event(**data):
            entry = dict(wall=time.time(), mono=time.monotonic(), **data)
            stream.write(json.dumps(entry, allow_nan=False)+'\n')
            stream.flush()
            print(json.dumps(entry, allow_nan=False), flush=True)
        def command(name, now):
            packet = posture_packet(g, now, name)
            r.send(packet)
            event(command=name, feedback=g.feedback)
        try:
            while True:
                now = time.monotonic()
                r.tick(now)
                f = g.feedback or {}
                if now-printed >= .5:
                    event(phase=phase, feedback=f, basic_age=now-g.basic_mono, motion_age=now-g.motion_mono,
                          telemetry_reason=g.check(now, moving=False))
                    printed = now
                if phase == 'preflight':
                    reason = posture_reason(g, now, 'LIE' if a.finish_standing else 'STAND')
                    if a.finish_standing and (f.get('state') != 17 or not finite_height(f)):
                        reason = reason or 'STANDING_FEEDBACK_REQUIRED'
                    stable_at = (stable_at or now) if reason is None else None
                    if stopping or now-phase_at > 8:
                        event(result='PREFLIGHT_STOPPED', reason=reason)
                        break
                    if stable_at and now-stable_at >= 2:
                        if not a.execute:
                            event(result='OBSERVATION_ONLY_PASSED')
                            break
                        attempted_stand = True
                        if a.finish_standing:
                            phase, phase_at = 'hold', now
                            event(result='EXISTING_STAND_CONFIRMED_NO_STAND_COMMAND')
                        else:
                            command('STAND', now)
                            phase, phase_at = 'standing', now
                elif phase == 'standing':
                    if g.check(now, moving=False) is None and f.get('state') == 17 and finite_height(f):
                        phase, phase_at = 'hold', now
                        event(result='STANDING_FEEDBACK_CONFIRMED')
                    # First stand after reboot caused native automatic homing
                    # (~14 s), then Idle settling and StandUp. Never retry STAND.
                    elif stopping or now-phase_at > 40:
                        event(result='STAND_NOT_CONFIRMED')
                        break
                elif phase == 'hold':
                    reason = posture_reason(g, now, 'LIE')
                    if reason is not None or f.get('state') != 17:
                        event(result='HOLD_INVALID', reason=reason or 'STATE_CHANGED')
                        break
                    if stopping or now-phase_at >= a.hold_seconds:
                        command('LIE', now)
                        requested_lie = True
                        phase, phase_at = 'lying', now
                        stable_at = None
                elif phase == 'lying':
                    if posture_reason(g, now, 'STAND') is None:
                        stable_at = stable_at or now
                        if now-stable_at >= 1:
                            event(result='RETURNED_LOW_IDLE', final_feedback=f, axis_commands_sent=0)
                            break
                    else:
                        stable_at = None
                    if now-phase_at > 8:
                        event(result='LIE_NOT_CONFIRMED')
                        break
                time.sleep(.01)
        finally:
            # Do not drop motor support or issue a blind lie under bad telemetry.
            if attempted_stand and not requested_lie:
                now = time.monotonic()
                r.tick(now)
                if posture_reason(g, now, 'LIE') is None:
                    command('LIE', now)
                    event(result='RECOVERY_LIE_SENT_NOT_YET_CONFIRMED')
                else:
                    event(result='NO_SAFE_AUTOMATIC_LIE', reason=posture_reason(g, now, 'LIE'))
            r.close()
            lock.close()


def finite_height(f):
    from native_nav.protocol import finite
    return finite(f.get('height')) and .20 <= f['height'] <= .65


if __name__ == '__main__':
    main()
