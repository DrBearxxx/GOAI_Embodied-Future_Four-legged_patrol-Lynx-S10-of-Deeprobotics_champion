"""Wake protocol and feedback gates with a mocked robot transport."""
import sys
import unittest
from types import SimpleNamespace as NS
from unified_control import Controller,Simulator


@unittest.skipUnless(sys.platform=='linux','Linux hardware adapter')
class WakeTests(unittest.TestCase):
    def make(self,reason='SLEEP_ACTIVE'):
        from hardware import Hardware
        h=Hardware.__new__(Hardware)
        h.health_reason=lambda now:reason
        h.proc=None;h.phase='IDLE';h.detail='';h.basic={'Sleep':True};h.basic_at=9.
        h.motion=NS(motion_state=NS(state=0),vel_x=0.,vel_y=0.,vel_yaw=0.)
        h.init_handover();h.sent=[];h.guardian=NS(quiesce=lambda now:None)
        h.send=lambda *args:h.sent.append(args)
        return h
    def test_wake_sends_only_factory_sleep_false(self):
        h=self.make();h.wake(10.)
        self.assertEqual(h.sent,[(0x100002,0x400002,{'Sleep':False})])
        self.assertEqual(h.phase,'WAIT_WAKE');self.assertFalse(h.owned)
        self.assertEqual(h.requested,'none');self.assertTrue(h.native_axes_inhibited)
    def test_other_safety_conditions_still_block_wake(self):
        for reason in ('HARD_STOP_ACTIVE','CHARGING','BASIC_STATUS_STALE','UNKNOWN_DDS_WRITER'):
            h=self.make(reason)
            with self.assertRaises(ValueError):h.wake(10.)
            self.assertFalse(h.sent)
    def test_wake_rejects_moving_robot(self):
        h=self.make();h.motion.vel_x=.2
        with self.assertRaises(ValueError):h.wake(10.)
        self.assertFalse(h.sent)
    def test_wake_rejects_policy_owner(self):
        h=self.make();h.proc=object()
        with self.assertRaises(ValueError):h.wake(10.)
        self.assertFalse(h.sent)
    def test_ack_or_old_status_cannot_confirm_wake(self):
        h=self.make();h.wake(10.);h.basic['Sleep']=False;h.advance_wake(10.2)
        self.assertEqual(h.phase,'WAIT_WAKE')
        h.basic_at=10.3;h.advance_wake(10.4)
        self.assertEqual(h.phase,'IDLE');self.assertEqual(len(h.sent),1)
    def test_timeout_never_retries_or_stands(self):
        h=self.make();h.wake(10.);h.advance_wake(19.)
        self.assertEqual(h.phase,'FAILED');self.assertEqual(len(h.sent),1)
    def test_already_awake_does_not_send(self):
        h=self.make(None);h.basic['Sleep']=False;h.wake(10.)
        self.assertFalse(h.sent)


class WakeControllerTests(unittest.TestCase):
    def test_wake_disarms_and_does_not_run(self):
        adapter=Simulator();c=Controller(adapter);c.enabled=True;c.axes=(.1,0.,0.)
        c.accept({'action':'wake'},10.)
        self.assertFalse(c.enabled);self.assertEqual(c.axes,(0.,0.,0.))


if __name__=='__main__':unittest.main()
