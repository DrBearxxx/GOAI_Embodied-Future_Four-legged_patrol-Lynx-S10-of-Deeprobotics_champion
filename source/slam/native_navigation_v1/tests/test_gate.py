import copy
import datetime
import hashlib
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav.protocol import Codec, velocity, operation, heartbeat
from native_nav.gate import Gate
from native_nav.client import Client
from native_nav.acceptance import verify, code_hash
from native_nav.gateway import Robot
from native_nav.protocol import body


def fresh(g, now):
    g.feedback = dict(state=17, basic_state=17, gait=0x3002, basic_gait=0x3002,
                      mode=1, speed=0., hard_stop=0, charge=0, sleep=False, wz=0.)
    g.basic_mono = g.motion_mono = g.env_mono = now
    g.environment = True


def request(g, now, kind='ARM', seq=1, v=None):
    m = dict(kind=kind, session=g.session, ticket=g.challenge(now), seq=seq)
    if v is not None:
        m['velocity'] = v
    return m


class ProtocolTests(unittest.TestCase):
    def test_real_axis_not_proportional(self):
        m = velocity(.1, 0, -.2)['PatrolDevice']
        self.assertEqual(m['Command'], 0x00110002)
        self.assertEqual(m['Items'], dict(X=.1, Y=0., Z=0., Roll=0., Pitch=0., Yaw=-.2))
        c = Codec()
        raw = c.encode(heartbeat())
        self.assertEqual(Codec.decode(raw)['Type'], 0x00100064)
        next_packet = c.encode(heartbeat())
        self.assertEqual(struct.unpack('<H', next_packet[6:8])[0], 1)
        self.assertEqual(next_packet[9], 1)

    def test_invalid_packets(self):
        packet = Codec().encode(heartbeat())
        for bad in (b'', packet[:-1], packet+b'0', b'bad!'+packet[4:], packet[:8]+b'\0'+packet[9:]):
            with self.assertRaises(ValueError):
                Codec.decode(bad)

    def test_limits_and_forbidden_operations(self):
        self.assertEqual(velocity(.20, 0, 0)['PatrolDevice']['Items']['X'], .20)
        for v in [(float('nan'), 0, 0), (True, 0, 0), (-.01, 0, 0), (.20001, 0, 0), (0, .01, 0), (0, 0, .21)]:
            with self.assertRaises(ValueError):
                velocity(*v)
        for name in ('STAIRS', 'ZERO_JOINTS', 'SOFT_ESTOP', 'SDK_ON'):
            with self.assertRaises(ValueError):
                operation(name)
        self.assertEqual(operation('FLAT')['PatrolDevice']['Items']['GaitParam'], 0x3002)


class GateTests(unittest.TestCase):
    def test_requested_point_two_ceiling(self):
        g = self.active()
        self.assertTrue(g.accept(request(g, 1.03, 'VELOCITY', 3, [.20, 0, 0]), 1.03)[0])
        self.assertFalse(g.accept(request(g, 1.04, 'VELOCITY', 4, [.20001, 0, 0]), 1.04)[0])
        self.assertFalse(g.armed)

    def active(self):
        g = Gate()
        fresh(g, 1.)
        self.assertTrue(g.accept(request(g, 1.), 1.)[0])
        self.assertTrue(g.accept(request(g, 1.01, 'VELOCITY', 2, [.08, 0., .1]), 1.02)[0])
        return g

    def test_lease_and_no_auto_rearm(self):
        g = self.active()
        self.assertEqual(g.tick(1.1), (.08, 0., .1))
        for t in (1.20, 1.26):
            fresh(g, t)
            c = g.tick(t)
        self.assertEqual(c, (0., 0., 0.))
        self.assertEqual(g.reason, 'COMMAND_LEASE_EXPIRED')
        fresh(g, 1.3)
        self.assertFalse(g.accept(request(g, 1.3, 'VELOCITY', 3, [.05, 0, 0]), 1.3)[0])

    def test_deadline_anchored_to_ticket_not_arrival(self):
        g = Gate()
        fresh(g, 1.)
        self.assertTrue(g.accept(request(g, 1.), 1.14)[0])
        self.assertEqual(g.deadline, 1.25)

    def test_replays_future_expired_wrong_session(self):
        for mutation, at in [(lambda m: None, 1.16), (lambda m: None, .99),
                             (lambda m: m.update(session='wrong'), 1.)]:
            g = Gate()
            fresh(g, 1.)
            m = request(g, 1.)
            mutation(m)
            self.assertFalse(g.accept(m, at)[0])
        g = self.active()
        m = request(g, 1.03, 'VELOCITY', 3, [.01, 0, 0])
        self.assertTrue(g.accept(m, 1.03)[0])
        self.assertFalse(g.accept(m, 1.04)[0])
        self.assertFalse(g.armed)
        self.assertFalse(g.accept(request(g, 1.05, 'ARM', 3), 1.05)[0])

    def test_robot_state_and_ownership(self):
        for field, value in [('hard_stop', 1), ('charge', 2), ('sleep', True), ('speed', float('nan')),
                             ('state', 100), ('basic_state', 4), ('mode', 0), ('gait', 0x1003)]:
            g = self.active()
            g.feedback[field] = value
            self.assertEqual(g.tick(1.1), (0., 0., 0.))
            self.assertFalse(g.armed, field)
        for field in ('basic_mono', 'motion_mono', 'env_mono'):
            g = self.active()
            setattr(g, field, -10.)
            self.assertEqual(g.tick(1.1), (0., 0., 0.))
        g = self.active()
        g.environment = False
        self.assertEqual(g.tick(1.1), (0., 0., 0.))

    def test_stalled_loop(self):
        g = self.active()
        g.tick(1.03)
        fresh(g, 1.20)
        self.assertEqual(g.tick(1.20), (0., 0., 0.))
        self.assertEqual(g.reason, 'GATE_LOOP_STALLED')

    def test_duplicate_timer_tick_does_not_invent_freshness(self):
        g = self.active()
        self.assertNotEqual(g.tick(1.03), (0., 0., 0.))
        self.assertNotEqual(g.tick(1.03), (0., 0., 0.))
        self.assertEqual(g.deadline, 1.26)
        self.assertEqual(g.tick(1.02), (0., 0., 0.))

    def test_operation_requires_stationary_and_correct_state(self):
        g = self.active()
        self.assertFalse(g.accept(request(g, 1.03, 'LIE', 3), 1.03)[0])
        fresh(g, 1.04)
        self.assertTrue(g.accept(request(g, 1.04, 'LIE', 4), 1.04)[0])
        self.assertFalse(g.accept(request(g, 1.05, 'STAND', 5), 1.05)[0])
        g.feedback.update(state=4, basic_state=4)
        self.assertTrue(g.accept(request(g, 1.06, 'STAND', 6), 1.06)[0])
        self.assertFalse(g.armed)

    def test_malformed_commands_and_stop_without_ticket(self):
        for change in [lambda m: m.update(seq=True), lambda m: m.update(velocity=[float('nan'), 0, 0]),
                       lambda m: m.update(velocity=[.05, .1, 0]), lambda m: m.pop('ticket')]:
            g = self.active()
            m = request(g, 1.03, 'VELOCITY', 3, [.01, 0, 0])
            change(m)
            self.assertFalse(g.accept(m, 1.03)[0])
        g = self.active()
        self.assertTrue(g.accept({'kind': 'STOP'}, 1.03)[0])
        self.assertFalse(g.armed)


class AcceptanceTests(unittest.TestCase):
    def test_default_is_locked(self):
        with self.assertRaises(ValueError):
            verify(ROOT/'acceptance.example.json', 'boot')

    def test_bound_to_code_boot_and_real_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            d = json.loads((ROOT/'acceptance.example.json').read_text())
            d.update(operator='test-fixture-not-physical-validation', body_boot_id='boot', gateway_sha256=code_hash())
            for k in list(d):
                if k.endswith('_verified'):
                    d[k] = True
            evidence = Path(temp)/'evidence.txt'
            evidence.write_text('unit test only')
            d.update(receiver_timeout_s=.2, measured_stop_distance_m=.03, verified_forward_speed_mps=.2, evidence_file=str(evidence),
                     evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
            p = Path(temp)/'acceptance.json'
            p.write_text(json.dumps(d))
            self.assertEqual(verify(p, 'boot')['operator'], d['operator'])
            for speed in (None, True, .1, float('nan'), 1.):
                p.write_text(json.dumps(dict(d, verified_forward_speed_mps=speed)))
                with self.assertRaisesRegex(ValueError, 'STOP_TEST_SPEED_NOT_COVERED'):
                    verify(p, 'boot')
            p.write_text(json.dumps(d))
            with self.assertRaises(ValueError):
                verify(p, 'new-boot')
            evidence.write_text('changed')
            with self.assertRaises(ValueError):
                verify(p, 'boot')


class StatusParsingTests(unittest.TestCase):
    def packet(self, kind, items, delay=0.):
        wall = time.time()-delay
        sec = int(wall)
        stamp = struct.pack('@ll', sec, int((wall-sec)*1e9))
        return (Codec().encode(body(kind, 0x00f00000, items)),
                [(socket.SOL_SOCKET, 35, stamp)], 0, None)

    def reader(self, packets):
        class FakeSocket:
            def recvmsg(self, *args):
                if not packets:
                    raise BlockingIOError()
                return packets.pop(0)
        g = Gate()
        r = Robot(g, 'offline')
        r.socket = FakeSocket()
        return g, r

    def test_full_status_and_kernel_timestamp(self):
        basic = dict(MotionState=17, Gait=0x3002, ControlUsageMode=1, Charge=0, HES=0, Sleep=False)
        motion = dict(MotionState=17, Gait=0x3002, LinearX=.01, LinearY=0., AngularZ=.05)
        packets = [self.packet(0x00100064, {'BasicStatus': basic}),
                   self.packet(0x00100001, {'MotionStatus': motion})]
        g, r = self.reader(packets)
        r.consume(10.)
        self.assertEqual(g.feedback['state'], 17)
        self.assertAlmostEqual(g.feedback['speed'], .01)
        self.assertLessEqual(g.motion_mono, 10.)

    def test_kernel_stale_and_invalid_fields_not_refreshed(self):
        motion = dict(MotionState=17, Gait=0x3002, LinearX=0., LinearY=0., AngularZ=0.)
        g, r = self.reader([self.packet(0x00100001, {'MotionStatus': motion}, delay=1.)])
        r.consume(10.)
        self.assertIsNone(g.feedback)
        motion['LinearX'] = 'not-a-number'
        g, r = self.reader([self.packet(0x00100001, {'MotionStatus': motion})])
        r.consume(10.)
        self.assertIsNone(g.feedback)
        self.assertEqual(g.reason, 'INVALID_ROBOT_DATAGRAM')

    def test_incomplete_status_fails_closed(self):
        g, r = self.reader([self.packet(0x00100064, {'BasicStatus': {'MotionState': 17}})])
        r.consume(10.)
        self.assertIsNotNone(g.check(10.))

    def test_live_notification_type_fractional_time_and_integer_sleep(self):
        # Real 2026-09-19 wire format, not invented enum coercion. State 0
        # remains 0 (not considered lying, standing, or a motion permit).
        basic = dict(MotionState=0, Gait=0, ControlUsageMode=0, Charge=0, HES=0, Sleep=0)
        motion = dict(MotionState=0, Gait=0, LinearX=0., LinearY=0., AngularZ=0.)
        packets = []
        for kind, items in [(0x00300064, {'BasicStatus': basic}), (0x00300001, {'MotionStatus': motion})]:
            packet = self.packet(kind, items)
            d = body(kind, 0x00f00000, items)
            d['PatrolDevice']['Time'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            packets.append((Codec().encode(d), *packet[1:]))
        g, r = self.reader(packets)
        r.consume(10.)
        self.assertIs(g.feedback['sleep'], False)
        self.assertEqual(g.feedback['state'], 0)
        g.environment = True
        g.env_mono = 10.
        self.assertEqual(g.check(10.), 'NOT_FACTORY_FLAT_NAVIGATION')

    def test_integer_sleep_rejects_unknown_and_string_values(self):
        for value in (2, -1, '0', None):
            basic = dict(MotionState=4, Gait=0, ControlUsageMode=0, Charge=0, HES=0, Sleep=value)
            g, r = self.reader([self.packet(0x00300064, {'BasicStatus': basic})])
            r.consume(10.)
            self.assertIsNone(g.feedback)

    def test_fractional_source_replay_does_not_refresh(self):
        m = dict(MotionState=17, Gait=0x3002, LinearX=0., LinearY=0., AngularZ=0.)
        packet = self.packet(0x00300001, {'MotionStatus': m})
        d = body(0x00300001, 0x00f00000, {'MotionStatus': m})
        d['PatrolDevice']['Time'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        packet = (Codec().encode(d), *packet[1:])
        g, r = self.reader([packet, packet])
        r.consume(10.)
        self.assertEqual(g.reason, 'ROBOT_SOURCE_TIME_REPLAY')


class TransportTests(unittest.TestCase):
    def start(self, simulation=True):
        cmd = [sys.executable, '-m', 'native_nav.gateway', '--port', '0']
        if simulation:
            cmd.append('--simulation')
        p = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(self.cleanup_process, p)
        line = p.stdout.readline()
        self.assertTrue(line, p.stderr.read() if p.poll() is not None else 'no startup')
        port = json.loads(line)['port']
        c = Client(port)
        self.addCleanup(c.close)
        return p, c, port

    def cleanup_process(self, p):
        p.terminate()
        try:
            p.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
            p.communicate(timeout=2)

    def fresh_status(self, c, predicate=lambda s: True, timeout=1.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            c.poll(time.monotonic())
            s = c.get(time.monotonic())
            if s and not c.ticket_used and predicate(s):
                return s
            time.sleep(.005)
        self.fail('status timeout: ' + str(c.error))

    def test_real_tcp_timeout_then_no_automatic_resume(self):
        p, c, port = self.start()
        self.assertEqual(self.fresh_status(c)['backend'], 'simulation')
        self.assertTrue(c.action('ARM', time.monotonic()))
        self.fresh_status(c, lambda s: s['armed'])
        self.assertTrue(c.action('VELOCITY', time.monotonic(), [.02, 0, 0]))
        self.fresh_status(c, lambda s: s['reason'] == 'FOLLOW')
        s = self.fresh_status(c, lambda s: not s['armed'], timeout=.6)
        self.assertEqual(s['reason'], 'COMMAND_LEASE_EXPIRED')
        self.assertTrue(c.action('VELOCITY', time.monotonic(), [.02, 0, 0]))
        s = self.fresh_status(c, lambda s: s['reason'] == 'EXPLICIT_ARM_REQUIRED')
        self.assertFalse(s['armed'])

    def test_disconnect_new_session_requires_arm(self):
        p, c, port = self.start()
        old = self.fresh_status(c)['session']
        c.action('ARM', time.monotonic())
        self.fresh_status(c, lambda s: s['armed'])
        c.close()
        time.sleep(.06)
        d = Client(port)
        self.addCleanup(d.close)
        s = self.fresh_status(d)
        self.assertNotEqual(old, s['session'])
        self.assertFalse(s['armed'])

    def test_continuous_stream_then_explicit_stop(self):
        p, c, port = self.start()
        self.fresh_status(c)
        c.action('ARM', time.monotonic())
        self.fresh_status(c, lambda s: s['armed'])
        deadline = time.monotonic()+.6
        count = 0
        while time.monotonic() < deadline:
            c.poll(time.monotonic())
            s = c.get(time.monotonic())
            if s and not c.ticket_used:
                self.assertTrue(s['armed'], s)
                self.assertTrue(c.action('VELOCITY', time.monotonic(), [.02, 0, .03]))
                count += 1
            time.sleep(.01)
        self.assertGreater(count, 5)
        c.stop()
        s = self.fresh_status(c, lambda s: not s['armed'])
        self.assertEqual(s['reason'], 'OPERATOR_STOP')

    def test_offline_cannot_send_motion(self):
        p, c, port = self.start(False)
        self.assertEqual(self.fresh_status(c)['backend'], 'offline')
        c.action('STAND', time.monotonic())
        self.fresh_status(c, lambda s: c.ack is not None)
        self.assertEqual(c.ack['reason'], 'MONITOR_ONLY')
        self.assertEqual(c.value['sent_motion'], 0)

    def test_late_response_invalidates(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        self.addCleanup(listener.close)
        c = Client(listener.getsockname()[1])
        self.addCleanup(c.close)
        peer, _ = listener.accept()
        self.addCleanup(peer.close)
        c.poll(1.)
        c.poll(1.16)
        self.assertTrue(c.closed)
        self.assertEqual(c.error, 'GATEWAY_RTT_EXPIRED')


if __name__ == '__main__':
    unittest.main(verbosity=2)
