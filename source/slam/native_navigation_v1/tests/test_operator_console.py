"""Terminal UI tests only: mock commands or a disposable tmux stdin echo sink."""
import contextlib
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav import operator_console as ui


class ConsoleTests(unittest.TestCase):
    def test_one_allowlisted_literal_line_only(self):
        with patch.object(ui, 'route_identity', return_value=123), patch.object(ui, 'tmux') as tm:
            self.assertEqual(ui.submit('%1', 123, '  stop '), 'STOP')
            tm.assert_called_once_with('send-keys', '-t', '%1', 'C-u', ';',
                                      'send-keys', '-t', '%1', '-l', 'STOP', ';',
                                      'send-keys', '-t', '%1', 'Enter')

    def test_invalid_or_multiple_commands_never_sent(self):
        for name in ('STAND\nARM', 'STAND; ARM', 'bash', 'ARM_TRIAL50', '', 'STOP\x1b'):
            with self.subTest(name=name), patch.object(ui, 'tmux') as tm:
                with self.assertRaises(ValueError): ui.submit('%1', 123, name)
                tm.assert_not_called()

    def test_replaced_process_never_receives_command(self):
        with patch.object(ui, 'route_identity', return_value=124), patch.object(ui, 'tmux') as tm:
            with self.assertRaises(ValueError): ui.submit('%1', 123, 'ARM')
            tm.assert_not_called()

    def test_dead_pane_and_copy_mode_rejected(self):
        for record in ('%1\t1\t123\t0\tpython', '%1\t0\t123\t1\tpython', '%2\t0\t123\t0\tpython'):
            with patch.object(ui, 'tmux', return_value=record):
                with self.assertRaises(ValueError): ui.route_identity('%1')

    def test_install_creates_input_only_and_is_idempotent(self):
        calls = []
        def fake(*args):
            calls.append(args)
            if args[0] == 'list-panes': return '%1\t0\t123\t0\tbash first_trial_pane.sh route'
            if args[0] == 'split-window': return '%2'
            return ''
        with patch.object(ui, 'tmux', side_effect=fake):
            self.assertEqual(ui.install(), '%2')
        self.assertFalse(any(x[0] in ('send-keys', 'respawn-pane', 'kill-pane') for x in calls))
        self.assertEqual(len([x for x in calls if x[0] == 'split-window']), 1)
        self.assertIn('8', next(x for x in calls if x[0] == 'split-window'))
        record = '%1\t0\t123\t0\troute\n%2\t0\t124\t0\tpython '+str(ui.ROOT/'scripts/route_console.py')+' --target-pane %1'
        with patch.object(ui, 'tmux', return_value=record) as tm:
            self.assertEqual(ui.install(), '%2')
            self.assertEqual([c.args[0] for c in tm.call_args_list], ['list-panes', 'select-pane'])

    def test_ambiguous_or_dead_install_does_not_change_panes(self):
        for record in ('%1\t1\t123\t0\troute', '%1\t0\t123\t0\troute\n%2\t0\t124\t0\tbash'):
            with patch.object(ui, 'tmux', return_value=record) as tm:
                with self.assertRaises(ValueError): ui.install()
                tm.assert_called_once()

    def test_startup_and_help_do_not_submit(self):
        with patch.object(sys, 'argv', ['console', '--target-pane', '%1']), \
             patch.object(sys.stdin, 'isatty', return_value=True), patch.dict(os.environ, {'TMUX': 'test'}), \
             patch('builtins.input', side_effect=['HELP', '', RuntimeError('test end')]), \
             patch.object(ui, 'submit') as send, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, 'test end'): ui.main()
            send.assert_not_called()

    def test_ctrl_c_requests_stop_and_keeps_console_open(self):
        with patch.object(sys, 'argv', ['console', '--target-pane', '%1']), \
             patch.object(sys.stdin, 'isatty', return_value=True), patch.dict(os.environ, {'TMUX': 'test'}), \
             patch('builtins.input', side_effect=[KeyboardInterrupt, 'QUIT']), \
             patch.object(ui, 'route_identity', return_value=123), \
             patch.object(ui, 'submit', side_effect=['STOP', 'QUIT']) as send, \
             patch.object(ui, 'wait_replies'), \
             contextlib.redirect_stdout(io.StringIO()):
            ui.main()
            self.assertEqual([x.args[2] for x in send.call_args_list], ['STOP', 'QUIT'])


@unittest.skipUnless(sys.platform == 'linux' and shutil.which('tmux'), 'Linux tmux integration')
class TmuxIntegrationTests(unittest.TestCase):
    def test_separate_input_and_exact_submission_no_robot(self):
        # Dedicated socket and temporary executable: cannot target any production
        # pane or connect to any robot. The fake route only echoes stdin.
        socket_name = 's10-ui-test-'+str(os.getpid())
        def tmux_test(*args):
            result = subprocess.run(['tmux', '-L', socket_name, *args], capture_output=True,
                                    text=True, timeout=3, check=True)
            return result.stdout.strip()
        with tempfile.TemporaryDirectory(prefix='s10-ui-test-') as temp:
            root = Path(temp)
            (root/'scripts').mkdir()
            shutil.copyfile(ROOT/'tests/fixtures/console_echo_sink.py', root/'scripts/route_ros.py')
            shutil.copyfile(ROOT/'tests/fixtures/console_test_frontend.py', root/'scripts/route_console.py')
            try:
                original = tmux_test('new-session', '-d', '-s', 'test-ui', '-n', 'route', '-x', '100', '-y', '30',
                                     '-P', '-F', '#{pane_id}', '-c', str(root),
                                     sys.executable, str(root/'scripts/route_ros.py'), '--execute')
                tmux_test('set-environment', '-g', 'S10_CONSOLE_TEST_SOURCE', str(ROOT))
                tmux_test('set-environment', '-g', 'S10_CONSOLE_TEST_PROJECT', str(root))
                with patch.object(ui, 'ROOT', root), patch.object(ui, 'tmux', side_effect=tmux_test):
                    lower = ui.install('test-ui')
                    self.assertNotEqual(lower, original)
                    self.assertEqual(ui.install('test-ui'), lower)
                    deadline = time.monotonic()+2.
                    while True:
                        try:
                            pid = ui.route_identity(original)
                            break
                        except ValueError:
                            if time.monotonic() >= deadline: raise
                            time.sleep(.02)
                    self.assertNotIn('ECHO:', tmux_test('capture-pane', '-p', '-t', original))
                    deadline = time.monotonic()+2.
                    while 'command>' not in tmux_test('capture-pane', '-p', '-t', lower):
                        if time.monotonic() >= deadline: self.fail('Actual input prompt did not render')
                        time.sleep(.02)
                    # Simulated typing in the LOWER pane must traverse the real
                    # input parser and pid check before reaching the echo sink.
                    tmux_test('send-keys', '-t', lower, '-l', 'STOP')
                    tmux_test('send-keys', '-t', lower, 'Enter')
                    deadline = time.monotonic()+2.
                    while 'ECHO:STOP' not in tmux_test('capture-pane', '-p', '-t', original):
                        if time.monotonic() >= deadline: self.fail('Literal command not received by echo sink')
                        time.sleep(.02)
                    self.assertNotIn('ECHO:STOP', tmux_test('capture-pane', '-p', '-t', lower))
                    deadline = time.monotonic()+2.
                    while 'ECHO_FIXTURE_NO_ROBOT' not in tmux_test('capture-pane', '-p', '-t', lower):
                        if time.monotonic() >= deadline: self.fail('Correlated response not shown below the input')
                        time.sleep(.02)
                    self.assertNotIn('ECHO_FIXTURE_NO_ROBOT', tmux_test('capture-pane', '-p', '-t', original))
                    self.assertIn('command>', tmux_test('capture-pane', '-p', '-t', lower))
                    self.assertEqual(tmux_test('display-message', '-p', '-t', lower, '#{pane_height}'), '8')
                    # A shell in the target pane must never receive ARM.
                    shell = tmux_test('new-window', '-d', '-t', 'test-ui', '-P', '-F', '#{pane_id}', 'bash')
                    with self.assertRaises(ValueError): ui.route_identity(shell)
            finally:
                subprocess.run(['tmux', '-L', socket_name, 'kill-server'], capture_output=True, timeout=3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
