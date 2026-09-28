import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from standing_handover import StandingHandover

class Process:
    pid=987654;code=None
    def poll(self):return self.code

class Fake(StandingHandover):
    def __init__(self):
        self.init_handover();self.proc=None;self.log=None;self.policy={};self.policy_at=-1e9
        self.commands=[];self.sent=[];self.replies=[];self.motion=17;self.motion_at=0.;self.health=None
        self.phase='READY';self.requested='basic';self.confirmed='basic';self.owned=True
        self.quiet=False;self.guardian=NS(quiesced=lambda now:self.quiet)
    def ms(self):return self.motion
    def stopped(self):return True
    def health_reason(self,now,check_policy=True):return self.health
    def command(self,c):self.commands.append(c)
    def send(self,*args):self.sent.append(args)
    def launch_policy(self,now):self.proc=Process()
    def quiesce_native_input(self,now):self.native_axes_inhibited=True
    def prepare_native(self,mode,now):self.phase='WAIT_GAIT'
    def fail_transition(self,detail):self.phase='FAILED';self.detail=detail
    def status(self,now,**fields):
        self.policy.update(dict(fault='0',frozen='0',preflight='ok',gateway_handover='1',handover_prepare_ready='1',profile='low'))
        self.policy.update(fields);self.policy_at=now;self.motion_at=now
    def report(self,phase,now,**fields):self.status(now,handover=phase,handover_token=self.handover_token,**fields)

class StandingTransferTests(unittest.TestCase):
    def to_sdk(self,h):
        h.begin_standing_waypoint('low',0);self.assertEqual(h.sent,[])
        h.report('idle',.1,mode='disarmed');h.advance_handover(.2)
        self.assertEqual(h.phase,'HANDOVER_PREPARE');self.assertEqual(h.sent,[])
        h.report('prepared',.3,mode='disarmed');h.advance_handover(.4)
        self.assertEqual(h.phase,'HANDOVER_COMMIT');self.assertEqual(h.sent,[])
        h.report('await_sdk',.5,mode='disarmed');h.advance_handover(.6)
        self.assertEqual(h.phase,'HANDOVER_QUIESCE');self.assertEqual(h.sent,[])
        self.assertTrue(h.native_axes_inhibited)
        h.report('await_sdk',.61,mode='disarmed');h.advance_handover(.62)
        self.assertEqual(h.sent,[])  # guardian has not acknowledged deactivation
        h.quiet=True;h.advance_handover(.63)
        self.assertEqual(h.phase,'HANDOVER_SDK');self.assertEqual(h.sent,[(0x100005,0x300002,{'SDKEnable':True,'Frequency':200})])
    def test_preload_prepare_commit_sdk_hold_no_auto_policy(self):
        h=Fake();self.to_sdk(h)
        h.motion=100;h.report('active',.7,mode='returning');h.advance_handover(.8)
        self.assertEqual(h.phase,'HANDOVER_HOLD')
        h.report('active',1.,mode='hold',stance_ready='1');h.advance_handover(1.1)
        self.assertEqual(h.phase,'READY');self.assertEqual(h.confirmed,'low')
        self.assertNotIn('policy',h.commands);self.assertNotIn('disarm',h.commands)
    def test_old_or_wrong_token_cannot_enable_sdk(self):
        h=Fake();h.begin_standing_waypoint('low',0);h.report('idle',.1,mode='disarmed');h.advance_handover(.2)
        h.status(.3,handover='prepared',handover_token='wrong');h.advance_handover(.4)
        self.assertEqual(h.phase,'HANDOVER_PREPARE');self.assertFalse(h.sent)
    def test_preload_timeout_does_not_toggle_sdk_or_kill_old_owner(self):
        h=Fake();h.begin_standing_waypoint('low',0);h.advance_handover(26)
        self.assertEqual(h.phase,'FAILED');self.assertFalse(h.sent);self.assertIsNotNone(h.proc)
    def test_profile_is_requested_once_while_waiting(self):
        h=Fake();h.begin_standing_waypoint('high',0);h.report('idle',.1,mode='disarmed');h.advance_handover(.2)
        h.advance_handover(.3);h.advance_handover(.4)
        self.assertEqual(h.commands,['cycle_policy']);self.assertEqual(h.phase,'HANDOVER_PROFILE')
    def to_native(self,h):
        h.proc=Process();h.motion=100;h.status(.1,mode='policy');h.begin_standing_native('stairs',.2)
        self.assertEqual(h.commands,['return']);self.assertFalse(h.sent)
        h.status(.3,mode='hold',stance_ready='1');h.advance_handover(.4)
        self.assertEqual(h.phase,'HANDOVER_RELEASE');self.assertFalse(h.sent)
        h.report('await_native',.5,mode='hold');h.advance_handover(.6)
        self.assertEqual(h.phase,'HANDOVER_NATIVE');self.assertEqual(h.sent[-1][2],{'SDKEnable':False,'Frequency':200})
    def test_native_release_keeps_process_until_real_native_and_removed_writer(self):
        h=Fake();self.to_native(h)
        with patch('standing_handover.os.killpg',create=True) as kill:
            h.report('await_native',.7,mode='hold');h.advance_handover(.8);kill.assert_not_called()
            h.motion=17;h.report('released',.9,mode='disarmed',command_publisher='1');h.advance_handover(1);kill.assert_not_called()
            h.report('released',1.1,mode='disarmed',command_publisher='0');h.advance_handover(1.2)
            kill.assert_called_once();self.assertEqual(h.phase,'HANDOVER_REAP')
        h.proc.code=0;h.health='DDS_STALE_GRAPH';h.advance_handover(1.3)
        self.assertIsNone(h.proc);self.assertEqual(h.phase,'HANDOVER_GRAPH')
        h.health=None;h.advance_handover(1.4);self.assertEqual(h.phase,'WAIT_GAIT')
    def test_release_timeout_never_kills_or_damps_support(self):
        h=Fake();self.to_native(h)
        with patch('standing_handover.os.killpg',create=True) as kill:
            h.advance_handover(8);kill.assert_not_called()
        self.assertEqual(h.phase,'FAILED');self.assertIsNotNone(h.proc)
        self.assertNotIn('disarm',h.commands);self.assertNotIn('damping',h.commands)
    def test_sdk_error_does_not_pretend_confirmation(self):
        h=Fake();self.to_sdk(h)
        h.replies=[dict(type=0x100005,command=0x300002,error_code=8,mono=.7)]
        h.advance_handover(.8);self.assertEqual(h.phase,'FAILED');self.assertIn('8',h.detail)
    def test_invalid_robot_feedback_blocks_next_sdk_operation(self):
        h=Fake();h.begin_standing_waypoint('low',0);h.report('idle',.1,mode='disarmed');h.advance_handover(.2)
        h.report('prepared',.3);h.health='DDS_UNKNOWN';h.advance_handover(.4)
        self.assertEqual(len(h.commands),1);self.assertFalse(h.sent)
    def test_hold_confirmation_does_not_require_nominal_joint_error(self):
        h=Fake();self.to_sdk(h);h.motion=100
        h.report('active',.7,mode='hold',stance_ready='0',stand_error='.30');h.advance_handover(.8)
        h.report('active',.9,mode='hold',stance_ready='0',stand_error='.30');h.advance_handover(1.)
        self.assertEqual(h.phase,'READY')
    def test_release_does_not_require_nominal_joint_error(self):
        h=Fake();h.proc=Process();h.motion=100
        h.status(.1,mode='hold',stance_ready='0',stand_error='.30');h.begin_standing_native('stairs',.2)
        h.status(.3,mode='hold',stance_ready='0',stand_error='.30');h.advance_handover(.4)
        self.assertEqual(h.phase,'HANDOVER_RELEASE');self.assertTrue(h.commands[-1].startswith('handover_release:'))
    def test_quiesce_timeout_cannot_submit_sdk_request(self):
        h=Fake();h.begin_standing_waypoint('low',0)
        h.handover_phase('QUIESCE',0,'wait');h.native_axes_inhibited=True
        h.advance_handover(2.1)
        self.assertEqual(h.phase,'FAILED');self.assertFalse(h.sent);self.assertTrue(h.native_axes_inhibited)
    def test_native_low_rate_joints_do_not_block_preparation(self):
        h=Fake();h.begin_standing_waypoint('low',0)
        h.report('idle',.1,mode='disarmed',preflight='joint_telemetry_stale',joint_age_ms='750',handover_prepare_ready='1')
        h.advance_handover(.2)
        self.assertEqual(h.phase,'HANDOVER_PREPARE');self.assertFalse(h.sent)
    def test_missing_preparation_readiness_cannot_request_sdk(self):
        h=Fake();h.begin_standing_waypoint('low',0)
        h.report('idle',.1,mode='disarmed',handover_prepare_ready='0');h.advance_handover(.2)
        self.assertEqual(h.phase,'HANDOVER_PRELOAD');self.assertFalse(h.commands);self.assertFalse(h.sent)

if __name__=='__main__':unittest.main()
