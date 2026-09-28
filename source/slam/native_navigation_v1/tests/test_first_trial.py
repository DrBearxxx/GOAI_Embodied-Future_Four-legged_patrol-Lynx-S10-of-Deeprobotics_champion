"""Synthetic commissioning and loopback-only fallback tests. No robot UDP port."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav.first_trial import FirstTrialGate, PROFILE
from native_nav.protocol import Codec
from native_nav.client import Client
from native_nav.zero_guardian import ZeroGuardian
from test_gate import fresh, request


def live(g, t, low=False):
    fresh(g, t)
    g.guardian_ready = True
    g.feedback.update(height=.08 if low else .40, roll=0., pitch=0.)
    if low:
        g.feedback.update(state=0, basic_state=0, mode=0, gait=0, basic_gait=0)


class FirstTrialTests(unittest.TestCase):
    def test_regular_stand_then_stable_standing_mode_nav(self):
        g = FirstTrialGate()
        for i in range(61):
            t = 1.+i*.04
            live(g, t, low=True)
            g.tick(t)
        self.assertTrue(g.accept(request(g, t, 'STAND'), t)[0])
        for i in range(1, 62):
            now = t+i*.04
            live(g, now)
            g.feedback.update(mode=0, gait=0x1001, basic_gait=0x1001)
            g.tick(now)
            if i == 2:
                self.assertFalse(g.accept(request(g, now, 'MODE_NAV', 2), now)[0])
        self.assertTrue(g.accept(request(g, now, 'MODE_NAV', 3), now)[0])
        self.assertFalse(g.accept(request(g, now, 'SDK_OFF', 4), now)[0])
        self.assertFalse(g.armed)
        self.assertIsNone(g.started)

    def test_standing_mode_change_rejects_tilt_movement_stale_and_estop(self):
        for field, value in (('speed', .04), ('wz', .06), ('roll', .3), ('hard_stop', 1), ('height', .1)):
            g = FirstTrialGate()
            for i in range(61):
                t = 1.+i*.04
                live(g, t)
                g.feedback['mode'] = 0
                g.tick(t)
            g.feedback[field] = value
            self.assertFalse(g.accept(request(g, t+.01, 'MODE_NAV'), t+.01)[0], field)
        live(g, t+.02)
        g.motion_mono = t-1.
        self.assertFalse(g.accept(request(g, t+.02, 'MODE_NAV', 2), t+.02)[0])

    def test_formal_policy_still_requires_lying_for_mode_change(self):
        from native_nav.gate import Gate
        g = Gate()
        fresh(g, 1.)
        self.assertFalse(g.accept(request(g, 1., 'MODE_NAV'), 1.)[0])

    def active(self):
        g = FirstTrialGate()
        live(g, 1.)
        g.tick(1.)
        self.assertTrue(g.accept(request(g, 1., 'ARM_TRIAL50'), 1.)[0])
        return g

    def test_ordinary_arm_cannot_select_trial_profile(self):
        g = FirstTrialGate()
        live(g, 1.)
        self.assertFalse(g.accept(request(g, 1., 'ARM'), 1.)[0])
        self.assertFalse(g.armed)
        self.assertIsNone(g.started)

    def test_default_no_guardian_and_missing_posture_block(self):
        g = FirstTrialGate()
        fresh(g, 1.)
        self.assertFalse(g.accept(request(g, 1., 'ARM_TRIAL50'), 1.)[0])
        g.guardian_ready = True
        self.assertEqual(g.check(1.), 'POSTURE_TELEMETRY_REQUIRED')

    def test_one_shot_survives_new_client_session(self):
        g = self.active()
        g.stop('CLIENT_DISCONNECTED')
        g.session = 'new-client'
        live(g, 1.04)
        self.assertFalse(g.accept(request(g, 1.04, 'ARM_TRIAL50', 2), 1.04)[0])
        self.assertEqual(g.tick(1.08), (0., 0., 0.))
        self.assertTrue(g.accept(request(g, 1.08, 'LIE', 3), 1.08)[0])
        self.assertFalse(g.status(1.08)['ready'])
        self.assertEqual(g.status(1.08)['first_trial']['acceptance'], 'UNVERIFIED')

    def test_real_time_command_and_feedback_budgets(self):
        for field, value in (('started', -60.), ('command_path', .50),
                             ('command_turn', 1.20), ('feedback_path', .60)):
            g = self.active()
            setattr(g, field, value)
            live(g, 1.04)
            self.assertEqual(g.tick(1.04), (0., 0., 0.))
            self.assertFalse(g.armed)
            self.assertIsNotNone(g.finished_reason)

    def test_budget_integrates_and_does_not_reset_on_new_commands(self):
        g = self.active()
        for i in range(100):
            t = 1.01+i*.05
            live(g, t)
            g.feedback['speed'] = .20
            g.accept(request(g, t, 'VELOCITY', i+2, [.20, 0, 0]), t)
            if not g.armed:
                break
        self.assertEqual(g.finished_reason, 'BODY_TRIAL_COMMAND_PATH')
        self.assertGreaterEqual(g.command_path, .50)
        self.assertLess(g.command_path, .52)

    def test_normal_limits_ticket_hes_stall_and_zero_guardian(self):
        for change in ('overspeed', 'hard_stop', 'guardian', 'stale', 'tilt'):
            g = self.active()
            live(g, 1.04)
            if change == 'overspeed':
                self.assertFalse(g.accept(request(g, 1.04, 'VELOCITY', 2, [.21, 0, 0]), 1.04)[0])
            else:
                if change == 'hard_stop': g.feedback['hard_stop'] = 1
                if change == 'guardian': g.guardian_ready = False
                if change == 'stale': g.motion_mono = 0.
                if change == 'tilt': g.feedback['roll'] = .30
                self.assertEqual(g.tick(1.04), (0., 0., 0.))
            self.assertFalse(g.armed)
        g = self.active()
        live(g, 1.30)
        self.assertEqual(g.tick(1.30), (0., 0., 0.))
        self.assertEqual(g.finished_reason, 'GATE_LOOP_STALLED')

    def test_low_idle_is_measured_not_state_alias_and_stand_once(self):
        g = FirstTrialGate()
        for i in range(61):
            t = 1.+i*.04
            live(g, t, low=True)
            g.tick(t)
        self.assertTrue(g.low_idle_ready(t))
        self.assertTrue(g.accept(request(g, t, 'SDK_OFF'), t)[0])
        self.assertEqual(g.feedback['state'], 0)
        self.assertTrue(g.accept(request(g, t, 'MODE_NAV', 2), t)[0])
        g.feedback['mode'] = 1
        self.assertTrue(g.accept(request(g, t, 'STAND', 3), t)[0])
        self.assertFalse(g.accept(request(g, t, 'STAND', 4), t)[0])
        self.assertFalse(g.armed)
        g.feedback['height'] = .40
        g.tick(t+.04)
        self.assertFalse(g.low_idle_ready(t+.04))

    def test_no_runtime_flag_weakens_formal_execute(self):
        p = subprocess.run([sys.executable, '-m', 'native_nav.gateway', '--first-trial'],
                           cwd=ROOT, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=5)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('interactive operator terminal', p.stderr)
        p = subprocess.run([sys.executable, '-m', 'native_nav.gateway', '--execute', '--first-trial'],
                           cwd=ROOT, capture_output=True, text=True, timeout=5)
        self.assertNotEqual(p.returncode, 0)

    def test_first_trial_loopback_profile_and_one_shot(self):
        process = subprocess.Popen([sys.executable, '-m', 'native_nav.gateway', '--simulate-first-trial', '--port', '0'],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        client = None
        try:
            start = json.loads(process.stdout.readline())
            self.assertEqual(start['gateway'], 'simulation')
            client = Client(start['port'])
            def status(predicate=lambda s: True):
                end = time.monotonic()+1.
                while time.monotonic() < end:
                    now = time.monotonic()
                    client.poll(now)
                    s = client.get(now)
                    if s and not client.ticket_used and predicate(s):
                        return s
                    time.sleep(.005)
                self.fail('loopback status timeout')
            s = status()
            self.assertEqual(s['first_trial']['schema'], PROFILE)
            client.action('ARM_TRIAL50', time.monotonic())
            status(lambda s: s['armed'])
            client.action('VELOCITY', time.monotonic(), [.16, 0, 0])
            status(lambda s: s['reason'] == 'FOLLOW')
            client.stop()
            status(lambda s: not s['armed'])
            client.action('ARM_TRIAL50', time.monotonic())
            status(lambda s: client.ack and client.ack['reason'] == 'TRIAL_FINISHED_RESTART_REQUIRED')
            self.assertFalse(client.value['armed'])
        finally:
            if client: client.close()
            process.terminate()
            process.communicate(timeout=3)

    def test_launcher_default_is_inert(self):
        p = subprocess.run([sys.executable, str(ROOT/'scripts/launch_first_trial.py')],
                           capture_output=True, text=True, timeout=5)
        self.assertEqual(p.returncode, 0, p.stderr)
        data = json.loads(p.stdout)
        self.assertEqual(data['mode'], 'DESCRIPTION_ONLY')
        self.assertFalse(data['automatic_stand'])
        self.assertFalse(data['automatic_arm'])

    def test_launcher_layout_without_starting_any_process(self):
        from scripts import launch_first_trial as launcher
        calls = []
        def fake_run(args, **kwargs):
            calls.append(args)
            code = 1 if args[:2] == ['tmux', 'has-session'] else 0
            return subprocess.CompletedProcess(args, code, '', '')
        with patch.object(sys, 'argv', ['launcher', '--start']), \
             patch.object(sys.stdin, 'isatty', return_value=True), \
             patch.object(launcher.shutil, 'which', return_value='/usr/bin/tmux'), \
             patch('native_nav.bootstrap.verify', return_value='map'), \
             patch('native_nav.gateway.ownership_check', return_value=True), \
             patch('native_nav.operator_console.install') as install_console, \
             patch.object(launcher, 'navigation_running', return_value=False), \
             patch.object(Path, 'is_file', return_value=True), \
             patch.object(launcher.subprocess, 'run', side_effect=fake_run):
            launcher.main()
        windows = [x for x in calls if x[:2] in (['tmux', 'new-session'], ['tmux', 'new-window'])]
        self.assertEqual([x[x.index('-n')+1] for x in windows], ['body', 'localizer', 'route'])
        install_console.assert_called_once_with('s10-first-trial')
        self.assertFalse(any('ARM' in x or 'STAND' in x for cmd in calls for x in cmd))


@unittest.skipUnless(sys.platform == 'linux' and hasattr(os, 'pidfd_open'), 'Linux pidfd required')
class GuardianTests(unittest.TestCase):
    def test_clean_unowned_close_sends_no_udp(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            sink.bind(('127.0.0.1', 0))
            sink.settimeout(.1)
            udp.connect(sink.getsockname())
            g = ZeroGuardian(udp)
            try:
                deadline = time.monotonic()+1.
                while time.monotonic() < deadline:
                    g.pulse(False, Codec())
                    if g.healthy(): break
                    time.sleep(.02)
                self.assertTrue(g.healthy())
            finally:
                g.close()
            with self.assertRaises(socket.timeout): sink.recv(4096)

    def guardian_failure(self, action):
        # Disposable child owns ONLY a random loopback UDP endpoint. The pidfd
        # targets that child, never the test runner or any robot process.
        code = '''import os,signal,socket,sys,time
from native_nav.zero_guardian import ZeroGuardian
from native_nav.protocol import Codec
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.connect(('127.0.0.1',int(sys.argv[1])))
g=ZeroGuardian(s);codec=Codec();end=time.monotonic()+1.
while time.monotonic()<end:
 g.pulse(True,codec)
 if g.healthy(require_owned=True): break
 time.sleep(.01)
assert g.healthy(require_owned=True)
if sys.argv[2]=='stall': os.kill(os.getpid(),signal.SIGSTOP)
else: os._exit(0)
time.sleep(5)
'''
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
            sink.bind(('127.0.0.1', 0))
            sink.settimeout(3.)
            p = subprocess.Popen([sys.executable, '-c', code, str(sink.getsockname()[1]), action],
                cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                message = Codec.decode(sink.recv(4096))
                self.assertEqual(message['Type'], 0x00100001)
                self.assertTrue(all(v == 0 for v in message['Items'].values()))
                out, err = p.communicate(timeout=5)
                self.assertIn('ZERO_GUARDIAN_FALLBACK', out, err)
                self.assertEqual(p.returncode, -signal.SIGKILL if action == 'stall' else 0)
            finally:
                if p.poll() is None:
                    p.kill()
                    p.communicate(timeout=5)

    def test_gateway_stall_kills_exact_child_then_zero_only(self):
        self.guardian_failure('stall')

    def test_gateway_death_zero_only(self):
        self.guardian_failure('exit')


if __name__ == '__main__':
    unittest.main(verbosity=2)
