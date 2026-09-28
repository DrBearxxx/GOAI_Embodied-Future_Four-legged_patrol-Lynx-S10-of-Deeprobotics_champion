import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav.command_replies import CommandReplies, format_reply, parse_line
from native_nav.operation_feedback import OperationFeedback


def status(**changes):
    f = dict(state=17, basic_state=17, mode=1, gait=0x3002, basic_gait=0x3002,
             speed=0., wz=0., height=.4, roll=0., pitch=0., hard_stop=0, charge=0, sleep=False)
    f.update(changes)
    return dict(feedback=f, basic_age=.02, motion_age=.01)


class EffectTests(unittest.TestCase):
    def test_each_effect_needs_post_ack_fresh_stable_feedback(self):
        for name in ('MODE_NAV', 'STAND', 'FLAT', 'LIE'):
            with self.subTest(name=name):
                c = OperationFeedback(name, 'a'*32, 1, 1.)
                s = status(state=0, basic_state=0, height=.08) if name == 'LIE' else status()
                self.assertIsNone(c.observe(s, 1.01, 1.02, 1))  # Basic sample predates ACK.
                for now in (1.04, 1.16, 1.28):
                    self.assertIsNone(c.observe(s, now, now, 1))
                self.assertEqual(c.observe(s, 1.32, 1.32, 1)[0], 'CONFIRMED')

    def test_old_stale_inconsistent_and_unavailable_do_not_confirm(self):
        for s in (None, dict(status(), basic_age=2.), status(basic_state=0), status(hard_stop=1),
                  status(mode=0), status(height=None), status(speed=.10), status(pitch=.3)):
            c = OperationFeedback('FLAT', 'a'*32, 1, 1.)
            for now in (1.1, 1.3, 1.5):
                self.assertIsNone(c.observe(s, now, now, 1))

    def test_cached_status_cannot_accumulate_dwell(self):
        c = OperationFeedback('STAND', 'a'*32, 1, 1.)
        for now in (1.1, 1.2, 1.3, 1.4):
            self.assertIsNone(c.observe(status(), 1.1, now, 1))

    def test_mode_reverts_during_dwell_resets_confirmation(self):
        c = OperationFeedback('MODE_NAV', 'a'*32, 1, 1.)
        self.assertIsNone(c.observe(status(), 1.1, 1.1, 1))
        self.assertIsNone(c.observe(status(mode=0), 1.2, 1.2, 1))
        self.assertIsNone(c.observe(status(), 1.4, 1.4, 1))

    def test_timeout_and_reconnect_are_unconfirmed(self):
        c = OperationFeedback('STAND', 'a'*32, 1, 1.)
        self.assertEqual(c.observe(None, None, 41.01, 1)[0], 'UNCONFIRMED')
        self.assertEqual(c.observe(status(), 1.2, 1.2, 2)[0], 'UNCONFIRMED')

    def test_ack_is_intermediate_then_matching_final_and_busy_clears(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'r.jsonl'
            r = CommandReplies(path)
            self.addCleanup(r.close)
            r.queued('MODE_NAV', 'a'*32, 1, 1, .9)
            r.poll([dict(seq=1, ok=True, reason='REQUESTED_MODE_NAV')], 1, 1.)
            self.assertTrue(r.busy)
            for now in (1.1, 1.24, 1.4):
                s = status()
                r.poll([], 1, now, s['feedback'], s, now)
            self.assertFalse(r.busy)
            events = [json.loads(x) for x in path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual([e['phase'] for e in events], ['QUEUED', 'ACK', 'CONFIRMED'])
            self.assertFalse(events[1]['final'])
            self.assertTrue(events[-1]['final'])
            self.assertEqual(events[-1]['request_id'], 'a'*32)
            r.close()

    def test_stop_cancels_wait_not_claiming_physical_action_cancelled(self):
        with tempfile.TemporaryDirectory() as temp:
            r = CommandReplies(Path(temp)/'r.jsonl')
            self.addCleanup(r.close)
            r.queued('STAND', 'a'*32, 1, 1, .9)
            r.poll([dict(seq=1, ok=True, reason='REQUESTED_STAND')], 1, 1.)
            r.cancel_waits('OPERATOR_STOP')
            self.assertFalse(r.busy)
            self.assertIn('动作可能仍在执行', r.latest_action['reason'])
            self.assertIsNone(r.latest_action['ok'])
            r.close()

    def test_status_parser_and_sdk_ack_do_not_claim_switch_readback(self):
        self.assertEqual(parse_line('STATUS')[0], 'STATUS')
        text = format_reply(dict(phase='ACK', command='SDK_OFF', ok=True, reason='REQUESTED_SDK_OFF'))
        self.assertIn('无 SDKEnable 回读', text)
        text = format_reply(dict(phase='ACK', command='FLAT', ok=False, reason='NAV_MODE_REQUIRED',
                                 feedback=status(mode=0)['feedback']))
        self.assertIn('mode=0', text)
        self.assertIn('MODE_NAV', text)


if __name__ == '__main__': unittest.main(verbosity=2)
