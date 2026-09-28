"""Delayed firmware replies are visible without changing motion decisions."""
import json,struct,sys,unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch


@unittest.skipUnless(sys.platform=='linux','Mocked Linux adapter')
class PolicyFeedbackTests(unittest.TestCase):
    def make(self):
        from test_hardware_contract import HardwareContract
        h=HardwareContract().make();h.rx=dict(datagrams=0,basic=0,unrelated=0,rejected=0)
        return h

    def reply(self,h,now,error):
        body=json.dumps(dict(PatrolDevice=dict(Type=0x300001,Command=0x300002,Items=dict(ErrorCode=error)))).encode()
        packet=struct.pack('<4sHHBBB5s',b'\xeb\x91\xeb\x90',len(body),42,1,42,1,bytes(5))+body
        h.sock=NS(recvmsg=Mock(side_effect=[(packet,[],0,None),BlockingIOError()]))
        with patch('hardware.received_monotonic',return_value=now):h.read(now)

    def test_e008_after_timeout_is_visible_with_actual_basic_gait(self):
        h=self.make();h.select_live('stairs',1.);h.advance_native(6.1)
        self.assertEqual(h.phase,'FAILED');sent=list(h.sent)
        self.reply(h,10.,0xE008);s=h.status(10.)
        self.assertEqual(s['gait_response']['error_code'],0xE008)
        self.assertEqual(s['actual_policy'],'basic');self.assertEqual(s['phase'],'FAILED')
        self.assertTrue(s['live_input_ready']);self.assertEqual(h.sent,sent)
        self.assertEqual(s['gait_response']['message'],42)

    def test_old_reply_is_not_shown_for_new_request(self):
        h=self.make();h.select_live('stairs',1.);self.reply(h,2.,0xE008)
        h.select_live('platform',3.)
        self.assertIsNone(h.status(3.)['gait_response'])

    def test_success_reply_cannot_claim_actual_target_without_feedback(self):
        h=self.make();h.select_live('stairs',1.);self.reply(h,2.,0);h.advance_native(2.)
        self.assertEqual(h.status(2.)['actual_policy'],'basic');self.assertEqual(h.phase,'WAIT_GAIT')

    def test_invalid_feedback_does_not_claim_actual_gait(self):
        h=self.make();h.healthy=lambda now:False
        self.assertEqual(h.status(1.)['actual_policy'],'none')


if __name__=='__main__':unittest.main()
