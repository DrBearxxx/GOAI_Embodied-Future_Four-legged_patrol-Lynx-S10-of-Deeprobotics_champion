"""Deterministic POLL/Enter races using socketpair, never robot interfaces."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav.client import Client
from native_nav.command_replies import CommandReplies, format_reply
from native_nav.operator_actions import OperatorActions, action_denial
from native_nav.reconnecting_client import ReconnectingClient


class OperatorActionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/'replies.jsonl'
        self.replies = CommandReplies(self.path)
        self.addCleanup(self.replies.close)
        connection, self.peer = socket.socketpair()
        self.peer.settimeout(.1)
        self.addCleanup(self.peer.close)
        self.client = ReconnectingClient()
        self.client.current = Client.from_socket(connection)
        self.client.connection_count = 1
        self.addCleanup(self.client.close)
        self.transport = self.client.current
        self.actions = OperatorActions(self.replies)
        self.client.poll(1.)
        self.read_wire()
        self.status(1.02)

    def read_wire(self):
        return [json.loads(x) for x in self.peer.recv(8192).splitlines()]

    def status(self, now):
        message = dict(kind='STATUS', id=self.transport.pending[0], session='session-a',
                       ticket='ticket-'+str(now), armed=False)
        self.peer.sendall((json.dumps(message)+'\n').encode())
        self.client.poll(now)

    def events(self):
        return [json.loads(x) for x in self.path.read_text(encoding='utf-8').splitlines()]

    def start_waiting(self, name='SDK_OFF', context=3):
        self.client.poll(1.04)
        wire = self.read_wire()
        self.assertEqual([m['kind'] for m in wire], ['POLL'])
        # Exact old failure: fresh STATUS exists but POLL has retired its ticket.
        self.assertIsNotNone(self.client.get(1.04))
        self.assertFalse(self.client.action(name, 1.04))
        self.assertTrue(self.actions.submit(name, 'a'*32, self.client, 1.04, context))
        self.assertIsNone(self.actions.tick(self.client, 1.04, context=context))
        self.assertEqual(self.transport.seq, 0)
        self.assertEqual(self.events()[-1]['phase'], 'WAIT_TICKET')

    def test_poll_race_waits_then_enqueues_exactly_once(self):
        self.start_waiting()
        self.status(1.08)
        self.assertEqual(self.actions.tick(self.client, 1.08, context=3), 'SDK_OFF')
        self.client.poll(1.12)
        wire = self.read_wire()
        actions = [m for m in wire if m['kind'] != 'POLL']
        self.assertEqual(len(actions), 1)
        self.assertEqual(actions[0]['kind'], 'SDK_OFF')
        self.assertEqual(actions[0]['ticket'], 'ticket-1.08')
        for now in (1.12, 1.16, 1.20):
            self.assertIsNone(self.actions.tick(self.client, now, context=3))
        self.assertEqual(self.transport.seq, 1)
        self.assertEqual([e['phase'] for e in self.events()], ['WAIT_TICKET', 'QUEUED'])

    def test_available_ticket_does_not_add_delay(self):
        self.assertTrue(self.actions.submit('STAND', 'a'*32, self.client, 1.02))
        self.assertEqual(self.actions.tick(self.client, 1.02), 'STAND')
        self.assertEqual([e['phase'] for e in self.events()], ['QUEUED'])

    def test_stop_quit_shutdown_cancel_without_wire_action(self):
        for reason in ('OPERATOR_STOP', 'QUIT', 'SHUTDOWN'):
            with self.subTest(reason=reason):
                self.assertTrue(self.actions.submit('STAND', 'a'*32, self.client, 1.02))
                self.actions.cancel(reason)
                self.assertIsNone(self.actions.tick(self.client, 1.03))
        self.assertEqual(self.transport.seq, 0)
        self.assertEqual(self.transport.outgoing, b'')
        self.assertTrue(all(e['phase'] == 'CANCELLED' for e in self.events()))

    def test_expiry_does_not_extend_freshness(self):
        self.start_waiting()
        self.actions.tick(self.client, 1.391, context=3)
        self.assertIsNone(self.actions.pending)
        self.assertIn('WAIT_EXPIRED', self.events()[-1]['reason'])
        self.assertEqual(self.transport.seq, 0)

    def test_late_status_closes_transport_and_cancels(self):
        self.start_waiting()
        self.status(1.20)
        self.assertIsNone(self.client.current)
        self.actions.tick(self.client, 1.20, context=3)
        self.assertIn('CONNECTION_CHANGED', self.events()[-1]['reason'])
        self.assertEqual(self.transport.seq, 0)

    def test_reconnection_does_not_replay_intent(self):
        self.start_waiting()
        self.client._failed(1.05, 'SIMULATED_DISCONNECT')
        a, b = socket.socketpair()
        self.addCleanup(b.close)
        self.client.current = Client.from_socket(a)
        self.client.current.session = 'session-b'
        self.client.connection_count += 1
        self.actions.tick(self.client, 1.08, context=3)
        self.assertIsNone(self.actions.pending)
        self.assertEqual(self.client.current.outgoing, b'')
        self.assertIn('CONNECTION_CHANGED', self.events()[-1]['reason'])

    def test_session_change_cancels_even_with_same_client(self):
        self.start_waiting()
        self.transport.session = 'restarted-session'
        self.actions.tick(self.client, 1.08, context=3)
        self.assertIn('CONNECTION_CHANGED', self.events()[-1]['reason'])
        self.assertEqual(self.transport.seq, 0)

    def test_ownership_change_cancels_with_fresh_ticket(self):
        self.start_waiting()
        self.status(1.08)
        self.actions.tick(self.client, 1.08, context=4)
        self.assertIn('CONTEXT_CHANGED', self.events()[-1]['reason'])
        self.assertEqual(self.transport.seq, 0)

    def test_new_ineligibility_cancels_before_send(self):
        self.start_waiting()
        self.status(1.08)
        self.actions.tick(self.client, 1.08, 'COMPETING_CONTROLLER', context=3)
        self.assertIn('COMPETING_CONTROLLER', self.events()[-1]['reason'])
        self.assertEqual(self.transport.seq, 0)

    def test_only_one_pending_then_only_one_unacknowledged_action(self):
        self.start_waiting()
        self.assertFalse(self.actions.submit('STAND', 'b'*32, self.client, 1.05, 3))
        self.assertEqual(self.actions.pending.name, 'SDK_OFF')
        self.status(1.08)
        self.actions.tick(self.client, 1.08, context=3)
        self.assertFalse(self.actions.submit('STAND', 'c'*32, self.client, 1.09, 3))
        self.assertEqual(self.transport.seq, 1)

    def test_ack_rejection_never_retries(self):
        self.actions.submit('SDK_OFF', 'a'*32, self.client, 1.02)
        self.actions.tick(self.client, 1.02)
        self.replies.poll([dict(seq=1, ok=False, reason='BODY_REJECTED')], 1, 1.03)
        self.actions.tick(self.client, 1.04)
        self.assertEqual(self.transport.seq, 1)
        self.assertEqual(self.events()[-1]['phase'], 'ACK')
        self.assertFalse(self.events()[-1]['ok'])

    def test_lost_ack_is_unconfirmed_not_retry(self):
        self.actions.submit('SDK_OFF', 'a'*32, self.client, 1.02)
        self.actions.tick(self.client, 1.02)
        self.replies.poll([], 1, 2.03)
        self.actions.tick(self.client, 2.04)
        self.assertEqual(self.transport.seq, 1)
        self.assertEqual(self.events()[-1]['phase'], 'UNCONFIRMED')

    def test_disconnected_submission_is_rejected_not_saved(self):
        self.client._failed(1.03, 'SIMULATED_DISCONNECT')
        self.assertFalse(self.actions.submit('SDK_OFF', 'a'*32, self.client, 1.04))
        self.assertIsNone(self.actions.pending)
        self.assertIn('NOT_CONNECTED', self.events()[-1]['reason'])

    def test_first_trial_arm_mapping_and_recheck(self):
        self.actions.first_trial = True
        self.start_waiting('ARM')
        self.status(1.08)
        self.assertEqual(self.actions.tick(self.client, 1.08, context=3), 'ARM')
        self.assertEqual(json.loads(self.transport.outgoing)['kind'], 'ARM_TRIAL50')
        self.assertFalse(self.events()[-1]['final'])

    def test_denial_preserves_environment_and_arm_guards(self):
        env = dict(mono=1., rl_service_active=False, other_nav_publishers=0,
                   other_named_joint_publishers=[], mode_publishers=0,
                   dds_ownership=dict(ok=True, reason=None, revision=3))
        self.assertIsNone(action_denial('SDK_OFF', False, None, {}, env, 1.01))
        self.assertIsNone(action_denial('ARM', False, None, dict(ready=True), env, 1.01))
        self.assertEqual(action_denial('ARM', False, None, dict(ready=False, state='NOT_READY'), env, 1.01), 'NOT_READY')
        self.assertIsNotNone(action_denial('STAND', True, None, {}, env, 1.01))
        self.assertIsNotNone(action_denial('STAND', False, 1., {}, env, 1.01))
        self.assertIsNotNone(action_denial('STAND', False, None, {}, env, 2.))
        self.assertIsNotNone(action_denial('STAND', False, None, {}, dict(env, rl_service_active=True), 1.01))

    def test_wait_receipt_is_nonfinal_and_explicitly_unsent(self):
        self.start_waiting()
        event = self.events()[-1]
        self.assertFalse(event['final'])
        self.assertIsNone(event['ok'])
        self.assertIn('尚未发送', format_reply(event))

    def test_clock_regression_cancels_intent(self):
        self.start_waiting()
        self.actions.tick(self.client, 1.03, context=3)
        self.assertIn('WAIT_EXPIRED', self.events()[-1]['reason'])
        self.assertEqual(self.transport.seq, 0)


class OperatorLoopbackTests(unittest.TestCase):
    def test_real_tcp_all_posture_commands_get_their_own_ack_in_simulation(self):
        # Simulation has no Robot UDP connection. The real TCP gate supplies
        # correlated ACKs; acceptance is independent of transport delivery.
        process = subprocess.Popen([sys.executable, '-m', 'native_nav.gateway', '--simulation', '--port', '0'],
                                   cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        client = None
        try:
            startup = json.loads(process.stdout.readline())
            client = ReconnectingClient(startup['port'])
            with tempfile.TemporaryDirectory() as temp:
                path = Path(temp)/'replies.jsonl'
                replies = CommandReplies(path)
                actions = OperatorActions(replies)
                try:
                    deadline = time.monotonic()+2.
                    while time.monotonic() < deadline and client.get(time.monotonic()) is None:
                        client.poll(time.monotonic())
                        time.sleep(.005)
                    self.assertEqual(client.get(time.monotonic())['backend'], 'simulation')
                    for index, name in enumerate(('SDK_OFF', 'MODE_NAV', 'STAND', 'FLAT', 'LIE'), 1):
                        now = time.monotonic()
                        client.poll(now)
                        self.assertTrue(actions.submit(name, f'{index:032x}', client, now))
                        deadline = now+1.5
                        while time.monotonic() < deadline:
                            now = time.monotonic()
                            client.poll(now)
                            actions.tick(client, now)
                            status = client.get(now)
                            replies.poll(client.drain_acks(), client.connection_count, now,
                                         (status or {}).get('feedback'), status, client.received)
                            if actions.pending is None and not replies.busy:
                                break
                            time.sleep(.04)
                        events = [json.loads(x) for x in path.read_text(encoding='utf-8').splitlines()]
                        event = events[-1]
                        self.assertEqual(event['request_id'], f'{index:032x}')
                        self.assertIn(event['phase'], ('ACK', 'CONFIRMED'))
                        self.assertIs(type(event['ok']), bool)
                        self.assertIsInstance(event['reason'], str)
                        self.assertEqual(client.current.seq, index)
                        self.assertEqual(client.get(time.monotonic())['sent_nonzero'], 0)
                finally:
                    replies.close()
        finally:
            if client:
                client.close()
            process.terminate()
            try:
                process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
