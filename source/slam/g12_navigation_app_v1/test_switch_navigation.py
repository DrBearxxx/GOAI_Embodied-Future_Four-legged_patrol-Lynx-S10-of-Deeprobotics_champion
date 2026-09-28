"""Navigation retains its mission but resets PID during a gateway switch pause."""
import unittest
from mission import Mission
from test_unified import FakeBridge,route,solution
from test_forward_entry import clear


class SwitchingBridge(FakeBridge):
    def __init__(self):super().__init__();self.switching=False
    def action(self,kind,rid,t,input_kind=None):
        super().action(kind,rid,t,input_kind)
        if kind.startswith('select:'):self.selected=kind[7:];self.switching=True
    def state(self):return dict(super().state(),policy_switch_paused=self.switching)
    def policy_ready(self,p,input_kind=None):return not self.switching and super().policy_ready(p,input_kind)


class SwitchNavigationTests(unittest.TestCase):
    def test_preset_change_pauses_without_losing_progress_or_pid_windup(self):
        b=SwitchingBridge();b.selected='basic_normal';m=Mission({'indoor':route()},b)
        m.plans.set_policy('indoor',[1,2],'stairs_normal')
        m.command({'action':'start'},solution(x=.5),10.)
        self.assertGreater(m.step(solution(x=.5),clear(10.),10.)['vx'],0.)
        out=m.step(solution(t=10.1,x=.85),clear(10.1),10.1)
        self.assertEqual(out['policy'],'stairs_normal');self.assertEqual(out['state'],'POLICY_SWITCH_PAUSE')
        self.assertEqual(out['vx'],0.);self.assertTrue(out['active'])
        progress=(m.navigator.target,list(m.navigator.reached))
        for t in (10.2,10.4,10.6,10.8):
            m.keepalive(m.navigator.run_id,t)
            out=m.step(solution(t=t,x=.85),clear(t),t)
            self.assertEqual(out['state'],'POLICY_SWITCH_PAUSE');self.assertEqual(out['vx'],0.)
            self.assertEqual((m.navigator.target,m.navigator.reached),progress)
            self.assertIsNone(m.navigator.tracker.previous);self.assertFalse(m.navigator.tracker.history)
        self.assertEqual(b.actions,['select:stairs_normal']);self.assertEqual(b.stops,0)
        b.switching=False;m.keepalive(m.navigator.run_id,10.9)
        out=m.step(solution(t=10.9,x=.85),clear(10.9),10.9)
        self.assertTrue(out['motion_authorized']);self.assertGreater(out['vx'],0.)
        self.assertEqual(m.owner,'auto');self.assertEqual(b.actions,['select:stairs_normal'])
        # A following waypoint with the same gait still passes continuously.
        m.keepalive(m.navigator.run_id,11.)
        out=m.step(solution(t=11.,x=1.8),clear(11.),11.)
        self.assertNotEqual(out['state'],'POLICY_SWITCH_PAUSE');self.assertGreater(out['vx'],0.)
        self.assertEqual(b.actions,['select:stairs_normal'])

    def test_manual_switch_zeros_then_resumes_current_sticks_without_enable(self):
        b=SwitchingBridge();b.input_kind='native_axes';m=Mission({'indoor':route()},b)
        m.owner='manual';m.execution=True;m.native_input_kind='native_axes'
        def rc(t,seq,axes):m.operator(dict(session='test',sample_seq=seq,sample_age_ms=0,fresh=True,axes=axes),t)
        rc(10.,1,[1.,-.8,.7]);m.step(None,{},10.)
        m.command(dict(action='manual',policy='platform'),None,10.1)
        out=m.step(None,{},10.1)
        self.assertEqual(out['state'],'POLICY_SWITCH_PAUSE');self.assertEqual(out['axes'],[0,0,0])
        self.assertTrue(out['active']);self.assertTrue(out['execution_requested'])
        b.switching=False;rc(10.7,2,[.5,.3,-.6]);out=m.step(None,{},10.7)
        self.assertEqual(out['axes'],[.5,.3,-.6]);self.assertTrue(out['motion_authorized'])
        self.assertEqual(b.actions,['select:platform']);self.assertEqual(b.stops,0)

    def test_operator_stop_during_switch_stays_paused_after_feedback(self):
        b=SwitchingBridge();b.selected='basic_normal';m=Mission({'indoor':route()},b)
        m.command({'action':'start'},solution(),10.);m.step(solution(),clear(10.),10.)
        m.command(dict(action='override',policy='stairs'),None,10.1)
        m.step(solution(t=10.1),clear(10.1),10.1)
        m.pause();b.switching=False
        out=m.step(solution(t=10.8),clear(10.8),10.8)
        self.assertEqual(out['owner'],'paused');self.assertFalse(out['active']);self.assertEqual(out['vx'],0.)

    def test_rejected_switch_does_not_automatically_retry(self):
        b=SwitchingBridge();b.selected='basic_normal';m=Mission({'indoor':route()},b)
        m.command({'action':'start'},solution(),10.);m.step(solution(),clear(10.),10.)
        m.command(dict(action='override',policy='stairs'),None,10.1)
        m.step(solution(t=10.1),clear(10.1),10.1)
        b.switching=False;b.enabled=False
        b.state=lambda:dict(requested='stairs',input_kind='velocity',enabled=False,
            policy_switch_error='firmware rejected',phase='FAILED')
        out=m.step(solution(t=10.8),clear(10.8),10.8)
        self.assertEqual(out['owner'],'paused');self.assertEqual(out['state'],'POLICY_SWITCH_FAILED')
        self.assertEqual(b.actions,['select:stairs']);self.assertEqual(out['vx'],0.)


if __name__=='__main__':unittest.main()
