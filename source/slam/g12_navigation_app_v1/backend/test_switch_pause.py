"""Exercise the sole writer against mocked hardware, including real wire fields."""
import sys,unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from core import MODES
from unified_control import Controller,POLICY_SWITCH_PAUSE_S


@unittest.skipUnless(sys.platform=='linux','Mocked Linux hardware adapter')
class SwitchPauseTests(unittest.TestCase):
    def make(self,kind='velocity'):
        import test_hardware_contract
        h=test_hardware_contract.HardwareContract().make()
        h.native_input_kind=kind;h.basic['ControlUsageMode']=0 if kind=='native_axes' else 1
        h.node=None;h.last_hb=100.;h.last_audit=100.;h.last_send=-1.
        h.rclpy=NS(spin_once=lambda *a,**k:None);h.read=lambda now:None
        h.guardian=NS(pulse=lambda *a:None)
        h.health_reason=lambda now,**kwargs:None
        c=Controller(h);c.enabled=True;c.native_input_kind=kind
        self.c=c;self.h=h;self.seq=0
        return c,h

    def accept(self,now,action='',axes=(1.2,-.65,1.5),kind=None):
        self.seq+=1
        self.c.accept(dict(action=action,axes=list(axes),input_kind=kind or self.c.native_input_kind,
            fresh=True,sample_seq=self.seq,sample_age_ms=0),now)

    def tick(self,now):
        self.h.last_send=now-1.
        with patch('hardware.time.monotonic',return_value=now):self.c.tick(now)

    def gait_requests(self):return [v for v in self.h.sent if v[1]==0x300002]

    def test_navigation_zeros_before_request_through_mode_reset_then_resumes(self):
        c,h=self.make();self.accept(1.);self.tick(1.)
        self.assertEqual(h.sent[-1][2]['X'],1.2)
        self.accept(1.01,'select:stairs');self.assertEqual(c.status(1.01)['output'],[0,0,0])
        for t in (1.02,1.22,1.42,1.51):
            self.accept(t);self.tick(t)
            self.assertFalse(self.gait_requests());self.assertFalse(any(h.sent[-1][2].values()))
            self.assertTrue(c.enabled);self.assertTrue(c.status(t)['policy_switch_paused'])
        self.accept(1.53);self.tick(1.53)
        self.assertEqual(self.gait_requests(),[(0x100001,0x300002,{'GaitParam':MODES['stairs']})])
        self.assertFalse(any(h.sent[-1][2].values()))
        h.motion.gait_state.gait=MODES['stairs'];h.motion_at=1.6;h.basic['ControlUsageMode']=0
        self.accept(1.61);self.tick(1.61)
        self.assertEqual(h.phase,'WAIT_INPUT_MODE');self.assertFalse(any(h.sent[-1][2].values()))
        h.basic['ControlUsageMode']=1;h.basic_at=h.motion_at=1.7
        self.accept(1.71);self.tick(1.71)
        self.assertTrue(c.enabled)
        self.accept(1.8,axes=(1.6,-.8,1.7));self.tick(1.8)
        self.assertFalse(c.status(1.8)['policy_switch_paused'])
        self.assertEqual(h.sent[-1],(0x100001,0x110002,dict(X=1.6,Y=-.8,Yaw=1.7,Z=0.,Roll=0.,Pitch=0.)))
        self.assertEqual(c.status(1.8)['output'],[1.6,-.8,1.7])

    def test_manual_rapid_choices_send_only_latest_and_resume_current_sticks(self):
        c,h=self.make('native_axes')
        for t,policy in ((1.,'stairs'),(1.2,'platform'),(1.4,'basic')):
            self.accept(t,'select:'+policy,(1.,-.8,.7));self.tick(t)
            self.assertTrue(c.enabled);self.assertFalse(any(h.sent[-1][2].values()))
            self.assertFalse(self.gait_requests())
        self.accept(1.51,axes=(.5,.4,-.3));self.tick(1.51)
        self.assertEqual([r[2]['GaitParam'] for r in self.gait_requests()],[MODES['basic']])
        h.motion_at=1.6
        self.accept(1.61,axes=(.5,.4,-.3));self.tick(1.61)
        self.accept(1.7,axes=(.5,.4,-.3));self.tick(1.7)
        self.assertEqual(h.sent[-1][2]['X'],.5);self.assertEqual(h.sent[-1][1],0x100002)
        self.assertFalse(any(r[1]==0x500002 for r in h.sent))

    def test_retarget_after_request_starts_another_zero_dwell(self):
        c,h=self.make();self.accept(1.,'select:stairs');self.tick(1.)
        for t in (1.2,1.4,1.51):self.accept(t);self.tick(t)
        self.accept(1.6,'select:platform');self.tick(1.6)
        for t in (1.8,2.,2.09):self.accept(t);self.tick(t)
        self.assertEqual(len(self.gait_requests()),1)
        self.accept(2.11);self.tick(2.11)
        self.assertEqual([r[2]['GaitParam'] for r in self.gait_requests()],[MODES['stairs'],MODES['platform']])
        self.assertFalse(any(h.sent[-1][2].values()))

    def test_stop_cancels_queued_selection_and_never_resumes(self):
        c,h=self.make();self.accept(1.,'select:stairs');self.tick(1.)
        c.stop('operator stop');self.tick(2.)
        self.assertFalse(self.gait_requests());self.assertFalse(c.enabled);self.assertIsNone(c.policy_switch)
        self.assertFalse(any(h.sent[-1][2].values()))

    def test_stale_input_during_pause_cannot_replay_old_axes(self):
        c,h=self.make();self.accept(1.,'select:stairs');self.tick(1.);self.tick(1.4)
        self.assertFalse(c.enabled);self.assertIsNone(c.policy_switch);self.assertFalse(self.gait_requests())
        self.assertFalse(any(h.sent[-1][2].values()))

    def test_feedback_failure_keeps_zero_and_reports_existing_reason(self):
        c,h=self.make();self.accept(1.,'select:stairs');self.tick(1.)
        for t in (1.2,1.4,1.51):self.accept(t);self.tick(t)
        h.gait_reply=dict(type=0x100002,command=0x300002,mono=1.6,error_code=57352)
        self.accept(1.7);self.tick(1.7)
        self.assertFalse(c.enabled);self.assertEqual(h.phase,'FAILED')
        self.assertFalse(any(h.sent[-1][2].values()));self.assertIn('57352',c.reason)

    def test_waypoint_low_to_high_stops_move_then_auto_resumes(self):
        c,h=self.make();h.motion.motion_state.state=100;h.proc=NS(poll=lambda:None)
        h.requested=h.confirmed='low';h.policy={'mode':'policy','profile':'low','live_profile_switch':'1'}
        h.policy_at=0.;published=[];h.publish_waypoints=lambda axes:published.append(tuple(axes))
        self.accept(1.,'select:high',(.8,0,.2),'waypoint_axes');self.tick(1.)
        self.assertFalse(h.commands);self.assertEqual(published[-1],(0,0,0))
        for t in (1.2,1.4,1.51):self.accept(t,axes=(.8,0,.2),kind='waypoint_axes');self.tick(t)
        self.assertEqual(h.commands,['select_profile:high']);self.assertTrue(c.enabled)
        self.assertEqual(published[-1],(0,0,0))
        h.policy['profile']='high';h.policy_at=1.6
        self.accept(1.61,axes=(.7,0,.1),kind='waypoint_axes');self.tick(1.61)
        self.accept(1.7,axes=(.7,0,.1),kind='waypoint_axes');self.tick(1.7)
        self.assertEqual(published[-1],(.7,0,.1));self.assertTrue(c.enabled)

    def test_cross_family_zeroes_before_existing_handover(self):
        c,h=self.make();loaded=[];h.launch_policy=lambda now:loaded.append(now)
        self.accept(1.,'select:low',(.8,0,.2),'waypoint_axes');self.tick(1.)
        self.assertFalse(loaded);self.assertFalse(c.enabled)
        self.tick(1.51)
        self.assertEqual(loaded,[1.51]);self.assertEqual(h.phase,'HANDOVER_PRELOAD')
        self.assertFalse(any(r[1]==0x300002 for r in h.sent))


if __name__=='__main__':unittest.main()
