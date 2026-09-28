"""Adapter boundary tests with mocked feedback; never instantiate robot transport."""
import unittest,sys
from types import SimpleNamespace as NS
from core import MODES

@unittest.skipUnless(sys.platform=='linux','Linux adapter; run on Orin without motors')
class HardwareContract(unittest.TestCase):
    def make(self,state=17,gait=0x3002,height=.4):
        from hardware import Hardware
        h=Hardware.__new__(Hardware)
        h.motion=NS(motion_state=NS(state=state),gait_state=NS(gait=gait),height=height,vel_x=0.,vel_y=0.,vel_yaw=0.)
        h.basic={'ControlUsageMode':1};h.proc=None;h.owned=True;h.phase='READY';h.confirmed='basic';h.requested='basic';h.policy={};h.detail='';h.audit={'ok':True,'reason':None}
        h.motion_at=0.;h.basic_at=0.;h.gait_sent_at=None;h.gait_reply=None;h.stand_sent_at=None;h.since=0.;h.replies=[]
        h.init_handover()
        h.guardian=NS(quiesce=lambda now:None,quiesced=lambda now:True)
        h.guard=lambda now:None;h.healthy=lambda now:True;h.health_reason=lambda now:None;h.sent=[];h.commands=[]
        h.send=lambda *args:h.sent.append(args);h.command=lambda word:h.commands.append(word)
        return h
    def test_native_switch_only_gait_no_sdk_toggle(self):
        h=self.make();h.select('stairs',0)
        self.assertEqual(h.sent,[(0x100001,0x300002,{'GaitParam':0x3003})]);self.assertEqual(h.confirmed,'none')
    def test_high_platform_id(self):
        h=self.make();h.select('platform',0);self.assertEqual(h.sent[-1][2],{'GaitParam':0x1002})
    def test_cross_family_standing_preloads_without_sdk_or_motor_request(self):
        h=self.make();loaded=[];h.launch_policy=lambda now:loaded.append(now)
        h.select('low',0)
        self.assertEqual(loaded,[0]);self.assertEqual(h.phase,'HANDOVER_PRELOAD');self.assertFalse(h.sent)
    def test_sdk_entry_waits_for_feedback(self):
        h=self.make(state=0,height=.08);h.select('low',0)
        self.assertEqual(h.phase,'WAIT_SDK_QUIET');self.assertEqual(h.confirmed,'none');self.assertFalse(h.sent)
        self.assertTrue(h.native_axes_inhibited)
    def test_profile_switch_does_not_return_or_stand(self):
        h=self.make(state=100);h.proc=object();h.policy={'mode':'policy','profile':'low','live_profile_switch':'1'};h.select('high',0)
        self.assertEqual(h.commands,['select_profile:high']);self.assertEqual(h.phase,'WAIT_PROFILE');self.assertEqual(h.confirmed,'none')
    def test_profile_switch_resets_recurrent_state_via_existing_node(self):
        h=self.make(state=100);h.proc=object();h.policy={'mode':'hold','profile':'low'};h.select('high',0)
        self.assertEqual(h.commands,['cycle_policy']);self.assertEqual(h.phase,'WAIT_PROFILE')
    def test_native_switch_while_measured_moving_needs_only_gait(self):
        h=self.make();h.motion.vel_x=.5
        h.select('stairs',0)
        self.assertEqual(h.sent,[(0x100001,0x300002,{'GaitParam':0x3003})])
    def test_sdk_ownership_switch_is_not_a_native_gait_change(self):
        h=self.make();h.motion.vel_x=.5
        with self.assertRaises(ValueError):h.select('low',0)
        self.assertFalse(h.sent)
    def test_near_zero_velocity_accepts_residual_motion(self):
        h=self.make();h.motion.vel_x=.08;h.motion.vel_y=.03;h.motion.vel_yaw=.12
        h.select('stairs',0);self.assertEqual(h.phase,'WAIT_GAIT')
    def test_near_zero_limits_are_finite_planar_and_yaw(self):
        h=self.make();h.motion.vel_x=.10;h.motion.vel_yaw=.15;self.assertTrue(h.stopped())
        h.motion.vel_y=.01;self.assertFalse(h.stopped())
        h.motion.vel_x=0.;h.motion.vel_y=0.;h.motion.vel_yaw=.151;self.assertFalse(h.stopped())
        h.motion.vel_yaw=float('nan');self.assertFalse(h.stopped())
    def test_native_sender_stays_off_after_sdk_request_even_on_old_native_feedback(self):
        h=self.make();h.requested='low';h.handover_source='native';h.phase='HANDOVER_COMMIT'
        self.assertTrue(h.native_output_active())
        h.quiesce_native_input(0);self.assertFalse(h.native_output_active())
        h.phase='HANDOVER_SDK';self.assertFalse(h.native_output_active())
        h.phase='FAILED';self.assertFalse(h.native_output_active())
        h.prepare_native('basic',1);self.assertTrue(h.native_output_active())
    def test_failed_unarmed_preload_can_return_to_native_without_sdk_cycle(self):
        from unittest.mock import patch
        h=self.make();h.phase='FAILED';h.proc=NS(pid=12345);h.policy={'mode':'disarmed'}
        with patch('hardware.os.killpg') as kill:
            h.select('basic',0);kill.assert_called_once()
        self.assertEqual(h.phase,'HANDOVER_REAP');self.assertFalse(h.sent)
    def test_failed_sdk_entry_at_rest_is_not_mistaken_for_ready_profile(self):
        from unittest.mock import patch
        h=self.make(state=0,height=.08);h.phase='FAILED';h.proc=NS(pid=12345)
        h.policy={'mode':'disarmed','profile':'low'}
        with patch('hardware.os.killpg') as kill:
            h.select('low',0);kill.assert_called_once()
        self.assertEqual(h.phase,'RELEASE_POLICY');self.assertEqual(h.confirmed,'none')
    def test_resting_sdk_request_is_not_followed_by_native_axis_in_same_tick(self):
        from unittest.mock import patch
        h=self.make(state=0,height=.08);h.select('low',100.)
        h.node=None;h.last_hb=100.;h.last_audit=100.;h.last_send=0.
        h.rclpy=NS(spin_once=lambda *a,**k:None);h.read=lambda now:None
        h.guardian=NS(pulse=lambda *a:None,quiesced=lambda now:True)
        with patch('hardware.time.monotonic',return_value=100.01):h.tick((0.,0.,0.),100.)
        self.assertEqual(h.phase,'WAIT_SDK')
        self.assertEqual(h.sent,[(0x100005,0x300002,{'SDKEnable':True,'Frequency':200})])
        with patch('hardware.time.monotonic',return_value=100.08):h.tick((0.,0.,0.),100.08)
        self.assertEqual(len(h.sent),1)  # old non-SDK telemetry cannot reopen sender
    def test_shutdown_during_sdk_entry_does_not_send_native_zero(self):
        h=self.make();h.native_axes_inhibited=True
        h.node=NS(destroy_node=lambda:None);h.rclpy=NS(shutdown=lambda:None)
        h.sock=NS(close=lambda:None);h.guardian=NS(close=lambda:None)
        h.close();self.assertFalse(h.sent)
    def test_color_confirmation_is_feedback_based(self):
        h=self.make();self.assertEqual(h.status(0)['confirmed'],'basic')
        h.motion.gait_state.gait=MODES['stairs'];self.assertEqual(h.status(0)['confirmed'],'none')
        h.motion.gait_state.gait=MODES['basic'];h.healthy=lambda now:False
        self.assertEqual(h.status(0)['confirmed'],'none')
    def test_native_ready_requires_navigation_velocity_mode(self):
        h=self.make();self.assertTrue(h.ready(0));h.basic['ControlUsageMode']=0;self.assertFalse(h.ready(0))
    def test_pause_does_not_send_damping(self):
        h=self.make();h.pause();self.assertFalse(h.sent);self.assertFalse(h.commands)
    def test_stop_cancels_standing_handover_before_sdk_request(self):
        h=self.make();h.phase='HANDOVER_PREPARE';h.handover_sdk_at=None
        h.policy={'handover':'prepared','handover_token':'test0001'};h.pause()
        self.assertEqual(h.phase,'FAILED');self.assertEqual(h.commands,['handover_cancel:test0001']);self.assertFalse(h.sent)
    def test_stop_after_sdk_request_keeps_handover_to_stable_hold(self):
        h=self.make();h.phase='HANDOVER_SDK';h.handover_sdk_at=1.;h.pause()
        self.assertEqual(h.phase,'HANDOVER_SDK');self.assertFalse(h.sent);self.assertFalse(h.commands)
    def test_lying_selection_waits_for_stand_without_any_robot_operation(self):
        h=self.make(state=0,gait=0,height=.08);h.select('basic',1)
        self.assertEqual(h.phase,'WAIT_STAND');self.assertFalse(h.sent)
        h.advance_native(100);self.assertEqual(h.phase,'WAIT_STAND')
    def test_stand_then_gait_once_then_feedback(self):
        h=self.make(state=0,gait=0,height=.08);h.select('basic',1);h.posture('stand',2)
        self.assertEqual(h.sent,[(0x100001,0x200002,{'MotionParam':1})])
        h.motion.motion_state.state=17;h.motion.gait_state.gait=0x1001;h.motion_at=3.
        h.advance_native(3.1);self.assertEqual(h.phase,'WAIT_GAIT')
        self.assertEqual(h.sent[-1],(0x100001,0x300002,{'GaitParam':0x3002}))
        h.advance_native(3.2);self.assertEqual(len(h.sent),2)
        h.motion.gait_state.gait=0x3002;h.motion_at=3.3;h.advance_native(3.4)
        self.assertEqual(h.phase,'WAIT_INPUT_MODE');self.assertEqual(h.confirmed,'basic')
        h.basic_at=3.5;h.motion_at=3.5;h.advance_native(3.6)
        self.assertEqual(h.phase,'READY')
    def test_gait_ack_is_not_confirmation(self):
        h=self.make(gait=0x1001);h.select('basic',1)
        h.gait_reply={'error_code':0};h.advance_native(2)
        self.assertEqual(h.phase,'WAIT_GAIT');self.assertEqual(h.confirmed,'none')
    def test_gait_rejection_is_explicit_and_heartbeat_cannot_hide_it(self):
        h=self.make(gait=0x1001);h.select('basic',1)
        h.gait_reply={'error_code':23};h.last_reply={'command':5,'error_code':0};h.advance_native(2)
        self.assertEqual(h.phase,'FAILED');self.assertIn('23',h.detail)
    def test_gait_timeout_and_explicit_standing_retry(self):
        h=self.make(gait=0x1001);h.select('basic',1);h.advance_native(6.1)
        self.assertEqual(h.phase,'FAILED');self.assertIn('0x1001',h.detail)
        h.select('stairs',7);self.assertEqual(h.phase,'WAIT_GAIT')
        self.assertEqual([x[2] for x in h.sent],[{'GaitParam':0x3002},{'GaitParam':0x3003}])
    def test_unconfirmed_standing_factory_mode_can_switch_without_lie_or_sdk(self):
        h=self.make(gait=0x1001);h.confirmed='none';h.phase='WAIT_GAIT';h.gait_sent_at=0
        h.select('platform',1);self.assertEqual(h.sent,[(0x100001,0x300002,{'GaitParam':0x1002})])
    def test_already_target_does_not_send_redundant_commands(self):
        h=self.make();h.select('basic',1)
        self.assertEqual(h.phase,'READY');self.assertEqual(h.confirmed,'basic');self.assertFalse(h.sent)
    def test_failure_does_not_block_lie_and_lie_cancels_pending_gait(self):
        h=self.make(gait=0x1001);h.phase='FAILED';h.posture('lie',1)
        self.assertEqual(h.phase,'WAIT_REST');self.assertEqual(h.sent[-1][2],{'MotionParam':4})
        h.motion.motion_state.state=0;h.motion_at=2.;h.advance_native(2.1)
        self.assertEqual(h.phase,'WAIT_STAND');self.assertIsNone(h.stand_sent_at)
        h.advance_native(50);self.assertEqual(len(h.sent),1)
    def test_usage_mode_is_selected_after_stand_and_gait(self):
        h=self.make(state=0,gait=0);h.basic['ControlUsageMode']=0;h.select('stairs',1)
        self.assertEqual(h.phase,'WAIT_STAND');self.assertFalse(h.sent)
        h.posture('stand',2);h.motion.motion_state.state=17;h.motion_at=3.;h.advance_native(3.1)
        self.assertEqual(h.sent[-1][2],{'GaitParam':0x3003})
        h.motion.gait_state.gait=0x3003;h.motion_at=3.2;h.advance_native(3.3)
        self.assertEqual(h.phase,'WAIT_INPUT_MODE');self.assertEqual(h.confirmed,'stairs')
        self.assertEqual(h.sent[-1],(0x100002,0x500002,{'Mode':1}))
    def test_target_gait_in_mode_zero_is_confirmed_not_timed_out(self):
        h=self.make(gait=0x1001);h.select('basic',1)
        h.basic['ControlUsageMode']=0;h.motion.gait_state.gait=0x3002;h.motion_at=2.
        h.advance_native(2.1)
        self.assertEqual(h.confirmed,'basic');self.assertEqual(h.status(2.1)['confirmed'],'basic')
        self.assertEqual(h.phase,'WAIT_INPUT_MODE');self.assertFalse(h.ready(2.1))
        h.basic['ControlUsageMode']=1;h.basic_at=2.2;h.motion_at=2.2;h.advance_native(2.3)
        self.assertTrue(h.ready(2.3));self.assertEqual(h.phase,'READY')
    def test_stale_mode_one_cannot_skip_post_gait_mode_transition(self):
        h=self.make(gait=0x1001);h.select('basic',1)
        h.motion.gait_state.gait=0x3002;h.motion_at=2.;h.advance_native(2.1)
        self.assertEqual(h.phase,'WAIT_INPUT_MODE');self.assertEqual(h.sent[-1][2],{'Mode':1})
        h.advance_native(2.2);self.assertEqual(h.phase,'WAIT_INPUT_MODE')
        self.assertEqual(len(h.sent),2)
    def test_velocity_mode_timeout_is_not_reported_as_gait_timeout(self):
        h=self.make();h.basic['ControlUsageMode']=0;h.select('basic',1)
        self.assertEqual(h.phase,'WAIT_INPUT_MODE');h.advance_native(6.1)
        self.assertEqual(h.phase,'FAILED');self.assertEqual(h.confirmed,'basic')
        self.assertIn('速度接口',h.detail);self.assertNotIn('步态确认超时',h.detail)
        self.assertEqual(h.sent,[(0x100002,0x500002,{'Mode':1})])
    def test_velocity_mode_rejection_does_not_resend_gait(self):
        h=self.make();h.basic['ControlUsageMode']=0;h.select('basic',1)
        h.replies=[dict(type=0x100002,command=0x500002,mono=1.1,error_code=23)]
        h.advance_native(1.2);self.assertEqual(h.phase,'FAILED')
        self.assertIn('ErrorCode=23',h.detail);self.assertEqual(len(h.sent),1)
    def test_mode_stage_never_requests_stand_or_lie(self):
        h=self.make();h.basic['ControlUsageMode']=0;h.select('basic',1)
        h.basic['ControlUsageMode']=1;h.basic_at=2.;h.motion_at=2.;h.advance_native(2.1)
        self.assertTrue(h.ready(2.1));self.assertEqual(h.sent,[(0x100002,0x500002,{'Mode':1})])
    def test_stale_pre_request_motion_cannot_confirm_gait(self):
        h=self.make(gait=0x1001);h.select('basic',1)
        h.motion.gait_state.gait=0x3002;h.motion_at=.5;h.advance_native(2)
        self.assertEqual(h.phase,'WAIT_GAIT')
    def test_stand_is_idempotent_when_already_standing(self):
        h=self.make();h.posture('stand',1);self.assertFalse(h.sent)
    def test_stand_is_not_resent_while_first_stand_is_pending(self):
        h=self.make(state=0,height=.08);h.phase='WAIT_STAND';h.posture('stand',1)
        h.posture('stand',1.1);h.posture('stand',1.2)
        self.assertEqual(h.sent,[(0x100001,0x200002,{'MotionParam':1})])
    def test_transient_feedback_delay_waits_but_cannot_send_gait(self):
        h=self.make(state=0,gait=0);h.select('basic',1);h.posture('stand',2)
        h.motion.motion_state.state=17;h.motion_at=3;h.healthy=lambda now:False
        h.advance_native(3.1);self.assertEqual(h.phase,'WAIT_STAND');self.assertEqual(len(h.sent),1)
        h.healthy=lambda now:True;h.advance_native(3.2);self.assertEqual(h.phase,'WAIT_GAIT')
    def test_tick_audits_using_time_after_callbacks(self):
        from unittest.mock import patch
        h=self.make(state=0);h.phase='IDLE';h.node=None;h.owned=False
        h.last_hb=100.;h.last_send=100.;h.last_audit=98.;h.motion_at=99.
        h.rclpy=NS(spin_once=lambda *a,**k:setattr(h,'motion_at',100.002))
        h.guardian=NS(pulse=lambda *a:None);h.read=lambda now:None
        checked=[];h.audit_now=lambda now:checked.append((now,h.motion_at))
        with patch('hardware.time.monotonic',return_value=100.003):h.tick((0.,0.,0.),100.)
        self.assertEqual(checked,[(100.003,100.002)])
        self.assertFalse(h.sent)

if __name__=='__main__':unittest.main(verbosity=2)
