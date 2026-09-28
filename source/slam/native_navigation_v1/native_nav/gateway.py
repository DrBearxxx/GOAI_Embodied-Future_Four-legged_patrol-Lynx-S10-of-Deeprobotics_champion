"""Robot-host gateway: loopback TCP over SSH, independent lease, explicit actions.

Default is OFFLINE (not even a robot heartbeat). --observe sends only heartbeat.
--execute requires physical acceptance evidence. --first-trial is a separate
one-shot commissioning envelope with UNVERIFIED acceptance and a zero companion.
--simulation and --simulate-first-trial have no robot socket.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import socket
import sys
import time

from .acceptance import verify, code_hash
from .gate import Gate
from .first_trial import FirstTrialGate
from .protocol import Codec, heartbeat, operation, velocity, finite


def ownership_check():
    """Own-process exclusion is supplementary; unknown UDP writers aren't discoverable."""
    try:
        for p in Path('/proc').iterdir():
            if not p.name.isdecimal() or int(p.name) == os.getpid():
                continue
            try:
                args = (p / 'cmdline').read_bytes().split(b'\0')
            except FileNotFoundError:
                continue
            # No whole command-line substring match (could match our documentation).
            names = [Path(a.decode(errors='replace')).name for a in args[:3] if a]
            if any(n in ('goai_node', 'rl_deploy', 'planner', 'charge_manager') for n in names):
                return False
        return True
    except (OSError, ValueError):
        return False


def live_preflight():
    if sys.platform != 'linux':
        raise ValueError('LIVE_GATEWAY_REQUIRES_ROBOT_LINUX_HOST')
    # Verifies local ownership of .103, not a reachable remote .103.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('10.21.33.103', 0))
    config = Path('/var/opt/robot/conf/robot_server/Network.toml').read_text()
    settings = re.findall(r'^\s*enableTls\s*=\s*(true|false)\s*(?:#.*)?$', config, re.M)
    if settings != ['false']:
        raise ValueError('PLAINTEXT_INTERFACE_NOT_CONFIGURED; no configuration was changed')


class Robot:
    def __init__(self, gate, mode):
        self.gate = gate
        self.mode = mode
        self.codec = Codec()
        self.guardian = None
        self.socket = None
        self.last_heartbeat = -1e9
        self.last_velocity = -1e9
        self.last_env = -1e9
        self.sent_nonzero = 0
        self.sent_motion = 0
        self.source_times = {}
        self.sim_state, self.sim_gait, self.sim_mode = 17, 0x3002, 1
        if mode in ('observe', 'execute', 'first_trial'):
            live_preflight()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.connect(('10.21.33.103', 30004))
            self.socket.setblocking(False)
            # Linux kernel arrival timestamps prevent queued datagrams renewing freshness.
            self.socket.setsockopt(socket.SOL_SOCKET, 35, 1)  # SO_TIMESTAMPNS
            if mode == 'first_trial':
                from .zero_guardian import ZeroGuardian
                self.guardian = ZeroGuardian(self.socket)

    def send(self, value):
        if self.socket is not None:
            self.socket.send(self.codec.encode(value))

    def action(self, name):
        if self.mode not in ('execute', 'simulation', 'first_trial'):
            raise ValueError('MONITOR_ONLY')
        value = operation(name)
        self.sent_motion += 1
        if self.mode == 'simulation':
            if name == 'STAND':
                self.sim_state = 17
                if isinstance(self.gate, FirstTrialGate):
                    self.sim_mode, self.sim_gait = 0, 0x1001
            elif name == 'LIE':
                self.sim_state = 4
            elif name == 'FLAT':
                self.sim_gait = 0x3002
            elif name == 'MODE_NAV':
                self.sim_mode = 1
        else:
            self.send(value)

    def consume(self, now):
        if self.socket is None:
            return
        import struct
        for _ in range(32):
            try:
                data, anc, flags, addr = self.socket.recvmsg(65536, 128)
            except BlockingIOError:
                break
            received = None
            for level, kind, stamp in anc:
                if level == socket.SOL_SOCKET and kind == 35:
                    sec, ns = struct.unpack('@ll', stamp[:struct.calcsize('@ll')])
                    received = sec + ns * 1e-9
            wall = time.time()
            if flags or received is None or not 0 <= wall - received <= .15:
                continue
            mono = now - (wall - received)
            try:
                d = self.codec.decode(data)
                stamp_text = d['Time']
                if not isinstance(stamp_text, str) or not re.fullmatch(r'\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:\.\d{1,6})?', stamp_text):
                    raise ValueError('INVALID_ROBOT_SOURCE_TIME')
                source = datetime.datetime.fromisoformat(stamp_text).timestamp()
                fractional_time = '.' in stamp_text
                if not (-.02 if fractional_time else -1) <= wall - source <= 2:
                    continue
                items = d['Items']
                f = dict(self.gate.feedback or {})
                # Guide uses 0x0010xxxx; live CS10100051 notifications on
                # 2026-09-19 use 0x0030xxxx. Keep a narrow explicit allowlist.
                basic = d['Type'] in (0x00100064, 0x00300064) and d['Command'] == 0x00f00000
                motion = d['Type'] in (0x00100001, 0x00300001) and d['Command'] == 0x00f00000
                if not (basic or motion):
                    continue
                stream = 'basic' if basic else 'motion'
                if fractional_time:
                    if source <= self.source_times.get(stream, -1e30):
                        self.gate.stop('ROBOT_SOURCE_TIME_REPLAY')
                        continue
                    if wall-source > (1.2 if basic else .30):
                        continue
                    self.source_times[stream] = source
                if basic:
                    b = items['BasicStatus']
                    for k in ('MotionState', 'Gait', 'ControlUsageMode', 'Charge', 'HES'):
                        if type(b[k]) is not int:
                            raise ValueError('INVALID_BASIC_STATUS')
                    sleep = b['Sleep']
                    if type(sleep) is int and sleep in (0, 1):
                        sleep = bool(sleep)
                    if type(sleep) is not bool:
                        raise ValueError('INVALID_SLEEP_STATUS')
                    f.update(basic_state=b['MotionState'], basic_gait=b['Gait'], mode=b['ControlUsageMode'],
                             charge=b['Charge'], hard_stop=b['HES'], sleep=sleep)
                    self.gate.basic_mono = mono
                elif motion:
                    m = items['MotionStatus']
                    if any(type(m[k]) is not int for k in ('MotionState', 'Gait')):
                        raise ValueError('INVALID_MOTION_STATE')
                    if not all(finite(m[k]) for k in ('LinearX', 'LinearY', 'AngularZ')):
                        raise ValueError('INVALID_MOTION_VELOCITY')
                    f.update(state=m['MotionState'], gait=m['Gait'], speed=math.hypot(m['LinearX'], m['LinearY']),
                             vx=m['LinearX'], vy=m['LinearY'], wz=m['AngularZ'])
                    # Optional posture telemetry must never retain an older value.
                    for remote, local in (('Height', 'height'), ('Roll', 'roll'), ('Pitch', 'pitch')):
                        f[local] = float(m[remote]) if finite(m.get(remote)) else None
                    if abs(m['AngularZ']) > .6:
                        self.gate.stop('UNEXPECTED_YAW_SPEED')
                    self.gate.motion_mono = mono
                else:
                    continue
                self.gate.feedback = f
            except (ValueError, KeyError, TypeError, UnicodeError, OverflowError):
                self.gate.stop('INVALID_ROBOT_DATAGRAM')

    def tick(self, now):
        if self.mode == 'simulation':
            f = dict(state=self.sim_state, basic_state=self.sim_state, gait=self.sim_gait,
                     basic_gait=self.sim_gait, mode=self.sim_mode, hard_stop=0, charge=0,
                     sleep=False, speed=abs(self.gate.command[0]), wz=self.gate.command[2],
                     height=.40 if self.sim_state == 17 else .08, roll=0., pitch=0.)
            self.gate.feedback = f
            self.gate.basic_mono = self.gate.motion_mono = now
            self.gate.environment = True
            self.gate.env_mono = now
            if isinstance(self.gate, FirstTrialGate):
                self.gate.guardian_ready = True  # simulator only; still labeled simulation
        elif self.socket is not None:
            if now - self.last_heartbeat >= 1.:
                self.send(heartbeat())
                self.last_heartbeat = now
            self.consume(now)
            if now - self.last_env >= .5:
                self.gate.environment = ownership_check()
                self.gate.env_mono = now
                self.last_env = now
        if self.guardian:
            self.guardian.pulse(self.gate.owned, self.codec)
            self.gate.guardian_ready = self.guardian.healthy()
        command = self.gate.tick(now)
        if any(command) and self.guardian and not self.guardian.healthy(require_owned=True):
            self.gate.stop('ZERO_GUARDIAN_NOT_READY')
            command = (0., 0., 0.)
        if self.mode in ('execute', 'simulation', 'first_trial') and self.gate.owned and now - self.last_velocity >= .1:
            # On safety stops keep sending zeros; never borrow GOAI's stream
            # withdrawal semantics. Receiver watchdog still required for our death.
            self.send(velocity(*command))
            self.sent_nonzero += int(any(command))
            self.last_velocity = now

    def close(self):
        self.gate.stop('GATEWAY_SHUTDOWN')
        if self.mode in ('execute', 'first_trial') and self.gate.owned:
            for _ in range(5):
                try:
                    if self.guardian:
                        self.guardian.pulse(True, self.codec)
                    self.send(velocity(0, 0, 0))
                except OSError:
                    pass
                time.sleep(.05)
        if self.socket:
            self.socket.close()
        if self.guardian:
            self.guardian.close()


def serve(args):
    trial = getattr(args, 'first_trial', False) or getattr(args, 'simulate_first_trial', False)
    mode = ('simulation' if args.simulation or getattr(args, 'simulate_first_trial', False)
            else 'first_trial' if getattr(args, 'first_trial', False)
            else 'execute' if args.execute else 'observe' if args.observe else 'offline')
    if mode == 'execute':
        verify(args.acceptance, Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    # One gateway per physical host, including monitor versus execution instances.
    lock = None
    if mode in ('execute', 'observe', 'first_trial'):
        import fcntl
        lock = open('/tmp/s10-native-navigation-gateway.lock', 'a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    gate = FirstTrialGate() if trial else Gate()
    robot = Robot(gate, mode)
    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('127.0.0.1', args.port))
    listener.listen(2)
    listener.setblocking(False)
    running = True

    def end(*unused):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, end)
    signal.signal(signal.SIGINT, end)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, end)
    print(json.dumps(dict(gateway=mode, port=listener.getsockname()[1], code_hash=code_hash(),
                         physical_acceptance='UNVERIFIED' if trial else 'SEE_EXECUTION_MODE')), flush=True)
    client, incoming, outgoing = None, b'', b''
    try:
        while running:
            now = time.monotonic()
            robot.tick(now)
            readers = [listener] + ([client] if client else [])
            ready, writable, _ = select.select(readers, [client] if client and outgoing else [], [], .01)
            if listener in ready:
                new, _ = listener.accept()
                new.setblocking(False)
                if client is not None:
                    new.close()
                else:
                    client = new
                    gate.stop('NEW_CONNECTION_REQUIRES_ARM')
                    gate.session = __import__('secrets').token_hex(16)
                    incoming = outgoing = b''
            if client and client in writable:
                try:
                    outgoing = outgoing[client.send(outgoing):]
                except (OSError, BlockingIOError):
                    gate.stop('CLIENT_WRITE_FAILED')
                    client.close()
                    client = None
            if client and client in ready:
                try:
                    chunk = client.recv(8192)
                    if not chunk:
                        raise ValueError('CLIENT_DISCONNECTED')
                    incoming += chunk
                    if len(incoming) > 16384:
                        raise ValueError('CLIENT_BACKLOG')
                    handled = 0
                    while b'\n' in incoming:
                        handled += 1
                        if handled > 8:
                            raise ValueError('CLIENT_FLOOD')
                        line, incoming = incoming.split(b'\n', 1)
                        m = json.loads(line)
                        now = time.monotonic()
                        if not isinstance(m, dict):
                            raise ValueError('INVALID_CLIENT_OBJECT')
                        if m.get('kind') == 'POLL':
                            identity = m.get('id')
                            if not isinstance(identity, str) or not 1 <= len(identity) <= 64:
                                raise ValueError('INVALID_POLL')
                            response = gate.status(now)
                            response.update(kind='STATUS', id=identity, backend=mode,
                                            sent_nonzero=robot.sent_nonzero, sent_motion=robot.sent_motion,
                                            code_hash=code_hash())
                        elif mode not in ('execute', 'simulation', 'first_trial'):
                            response = dict(kind='ACK', ok=False, reason='MONITOR_ONLY')
                        else:
                            ok, reason = gate.accept(m, now)
                            if ok and m['kind'] in ('SDK_OFF', 'MODE_NAV', 'STAND', 'FLAT', 'LIE'):
                                robot.action(m['kind'])
                            response = dict(kind='ACK', ok=ok, reason=reason, seq=m.get('seq'))
                        outgoing += (json.dumps(response, allow_nan=False) + '\n').encode()
                        if len(outgoing) > 32768:
                            raise ValueError('CLIENT_NOT_READING')
                except (OSError, ValueError, KeyError, TypeError, UnicodeError):
                    gate.stop('CLIENT_INVALID_OR_DISCONNECTED')
                    client.close()
                    client = None
                    incoming = outgoing = b''
    finally:
        robot.close()
        if client:
            client.close()
        listener.close()
        if lock:
            lock.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    group = p.add_mutually_exclusive_group()
    group.add_argument('--observe', action='store_true', help='Robot heartbeat only; may affect handset ownership')
    group.add_argument('--execute', action='store_true')
    group.add_argument('--simulation', action='store_true', help='No hardware sockets, ideal fake state')
    group.add_argument('--first-trial', action='store_true', help='Operator-supervised single 50 cm commissioning; acceptance remains UNVERIFIED')
    group.add_argument('--simulate-first-trial', action='store_true', help='One-shot profile in simulation; no robot socket')
    p.add_argument('--acceptance', default='acceptance.json')
    p.add_argument('--port', type=int, default=18891)
    args = p.parse_args()
    if not 0 <= args.port <= 65535:
        p.error('Invalid port')
    if args.first_trial and (sys.platform != 'linux' or not sys.stdin.isatty()):
        p.error('First trial requires Linux and an interactive operator terminal')
    serve(args)


if __name__ == '__main__':
    main()
