import json
import os
from pathlib import Path
import sys
import tempfile
import time
import socket
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav.command_replies import CommandReplies, parse_line, wait_replies
from native_nav.client import Client

class ReplyTests(unittest.TestCase):
    def test_multiple_acks_in_one_receive_are_preserved(self):
        a, b = socket.socketpair()
        client = Client.from_socket(a)
        try:
            b.sendall(b'{"kind":"ACK","seq":5,"ok":true,"reason":"SDK_OFF"}\n'
                      b'{"kind":"ACK","seq":6,"ok":true,"reason":"VELOCITY"}\n')
            client.poll(time.monotonic())
            self.assertEqual([x['seq'] for x in client.acks], [5, 6])
        finally:
            client.close()
            b.close()

    def test_request_parser_keeps_whitelist(self):
        self.assertEqual(parse_line('sdk_off '+'a'*32), ('SDK_OFF', 'a'*32))
        self.assertEqual(parse_line('STOP')[0], 'STOP')
        for text in ('STAND ARM', 'SDK_OFF;ARM', 'ARM xx', 'ARM '+'a'*32+' extra'):
            with self.assertRaises(ValueError): parse_line(text)

    def test_ack_is_correlated_not_last_ack(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'events.jsonl'
            r = CommandReplies(path)
            now = time.monotonic()
            r.queued('SDK_OFF', 'a'*32, 5, 1, now)
            r.poll([dict(seq=4, ok=True, reason='wrong'), dict(seq=5, ok=False, reason='REJECT'),
                    dict(seq=6, ok=True, reason='velocity')], 1, now+.1)
            r.close()
            events = [json.loads(x) for x in path.read_text().splitlines()]
            self.assertEqual([x['phase'] for x in events], ['QUEUED', 'ACK'])
            self.assertEqual(events[-1]['reason'], 'REJECT')
            self.assertFalse(events[-1]['ok'])

    def test_lost_ack_and_reconnect_never_claim_success(self):
        with tempfile.TemporaryDirectory() as root:
            for connection, age in ((2, .1), (1, 2.)):
                r = CommandReplies(Path(root)/'events.jsonl')
                r.queued('STAND', 'a'*32, 5, 1, 1.)
                r.poll([], connection, 1.+age)
                self.assertFalse(r.pending)
                r.close()
            event = json.loads((Path(root)/'events.jsonl').read_text().splitlines()[-1])
            self.assertIsNone(event['ok'])
            self.assertEqual(event['phase'], 'UNCONFIRMED')

    def test_arm_ack_is_not_final_until_route_handshake(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root)/'events.jsonl'
            r = CommandReplies(p)
            r.queued('ARM', 'a'*32, 1, 1, 1.)
            r.poll([dict(seq=1, ok=True, reason='ARM')], 1, 1.1)
            r.arm_result(False, 'ROUTE_NOT_READY')
            r.close()
            e = [json.loads(x) for x in p.read_text().splitlines()]
            self.assertFalse(e[-2]['final'])
            self.assertEqual(e[-1]['phase'], 'ARM_RESULT')
            self.assertFalse(e[-1]['ok'])

    def test_reader_filters_old_pid_and_request_and_tolerates_partial_line(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root)/'events.jsonl'
            r = CommandReplies(p)
            old = time.monotonic()
            r.emit('SDK_OFF', 'b'*32, 'ACK', 'OTHER', True)
            r.emit('SDK_OFF', 'a'*32, 'REJECTED', 'THIS_REQUEST', False)
            r.close()
            out = []
            self.assertTrue(wait_replies(p, os.getpid(), 'a'*32, old, out.append, timeout=.1))
            self.assertEqual(len(out), 1)
            self.assertIn('THIS_REQUEST', out[0])
            self.assertNotIn('OTHER', out[0])
            self.assertFalse(wait_replies(p, os.getpid()+1, 'a'*32, old, lambda x: None, timeout=.01))

if __name__ == '__main__': unittest.main(verbosity=2)
