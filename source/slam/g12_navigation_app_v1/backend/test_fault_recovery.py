"""Offline recovery regression: no ROS construction, sockets, or robot writes."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from hardware import Hardware

class FaultRecovery(unittest.TestCase):
    def make(self,state=100):
        from test_hardware_contract import HardwareContract
        h=HardwareContract().make(state=state)
        h.policy=dict(mode='damping',fault='0',frozen='0',profile='high_v2',command_publisher='1',live_profile_switch='1',fault_semantics='current_conditions')
        h.policy_at=h.motion_at=10.;h.phase='READY';h.requested='high_v2';h.confirmed='high_v2'
        h.proc=NS(pid=123456,poll=lambda:None);h.log=None;h.events=[]
        h.record=lambda event,**fields:h.events.append((event,fields))
        return h
    def test_recovered_fault_keeps_stopped_and_does_not_stand_or_run(self):
        h=self.make();h.reconcile_policy(10.1)
        self.assertEqual(h.phase,'DAMPING');self.assertEqual(h.confirmed,'none')
        self.assertIn('已清除',h.detail);self.assertTrue(h.live_input_lost(10.1))
        self.assertFalse(h.commands);self.assertFalse(h.sent)
    def test_active_reason_and_manual_damping_remain_distinct(self):
        h=self.make();h.policy.update(fault='1',fault_reason='motor_temperature_limit')
        h.reconcile_policy(10.1);self.assertIn('motor_temperature_limit',h.detail)
        h.policy.update(fault='0',frozen='1');h.reconcile_policy(10.1)
        self.assertIn('人工阻尼',h.detail);self.assertFalse(h.commands)
    def test_choose_model_in_recovered_damping(self):
        h=self.make();h.reconcile_policy(10.1);h.select('low_v5',10.1)
        self.assertEqual(h.commands,['select_profile:low_v5'])
        self.assertEqual(h.phase,'WAIT_PROFILE');self.assertFalse(h.sent)
    def test_choose_model_during_manual_damping_does_not_unfreeze(self):
        h=self.make();h.phase='DAMPING';h.policy['frozen']='1'
        h.health_reason=lambda now:'POLICY_FROZEN'
        h.select('low_v5',10.1);self.assertEqual(h.commands,['select_profile:low_v5'])
    def test_one_explicit_stand_uses_existing_reset_flow(self):
        h=self.make();h.health_reason=lambda *a,**kw:None
        h.reconcile_policy(10.1);h.posture('stand',10.1)
        self.assertEqual(h.commands,['reset']);self.assertEqual(h.phase,'DAMPING_RESET')
        h.reconcile_policy(10.2);self.assertEqual(h.phase,'DAMPING_RESET')
        h.policy.update(mode='disarmed');h.policy_at=10.3
        h.advance_damping(10.3);self.assertEqual(h.commands,['reset','stand'])
    def test_native_mode_selection_from_sdk_damping_at_rest(self):
        h=self.make();h.phase='DAMPING';h.motion.height=.1
        h.select('basic_normal',10.1)
        self.assertEqual(h.phase,'RECOVERY_SELECT');self.assertEqual(h.commands,['reset'])
        h.policy.update(mode='disarmed');h.policy_at=10.2
        with patch('hardware.os.killpg') as kill:
            h.reconcile_policy(10.2);kill.assert_called_once()
        self.assertEqual(h.phase,'RELEASE_POLICY');self.assertEqual(h.requested,'basic_normal')
        self.assertFalse(h.sent)  # SDK exit waits for child to exit, never auto-stands
    def test_native_release_reaps_only_acknowledged_nonpublisher(self):
        h=self.make(state=0);h.policy.update(mode='disarmed',native_released='1',command_publisher='0',fault='1')
        with patch('policy_recovery.os.killpg') as kill:
            h.reconcile_policy(10.1);kill.assert_called_once_with(h.proc.pid,2)
        self.assertEqual(h.phase,'RECOVERY_REAP');self.assertFalse(h.owned)
        h.proc.poll=lambda:0;h.reconcile_policy(10.2)
        self.assertIsNone(h.proc);self.assertEqual(h.policy,{})
        self.assertEqual(h.phase,'IDLE');self.assertFalse(h.sent);self.assertFalse(h.commands)
        h.select('basic_normal',10.2);self.assertEqual(h.requested,'basic_normal')
    def test_native_release_never_kills_unacknowledged_or_supporting_node(self):
        for changes in ({},dict(native_released='1'),dict(native_released='1',mode='disarmed'),
                        dict(native_released='1',command_publisher='0')):
            h=self.make(state=17);h.policy.update(changes)
            with patch('policy_recovery.os.killpg') as kill:h.reconcile_policy(10.1);kill.assert_not_called()
    def test_stale_release_does_not_retire_node(self):
        for stale in ('motion_at','policy_at'):
            h=self.make(state=17);h.policy.update(mode='disarmed',native_released='1',command_publisher='0')
            setattr(h,stale,5.)
            with patch('policy_recovery.os.killpg') as kill:h.reconcile_policy(10.1);kill.assert_not_called()
    def test_expected_handover_keeps_existing_protocol(self):
        h=self.make(state=17);h.phase='HANDOVER_NATIVE'
        h.policy.update(mode='disarmed',native_released='1',command_publisher='0')
        with patch('policy_recovery.os.killpg') as kill:h.reconcile_policy(10.1);kill.assert_not_called()
        self.assertEqual(h.phase,'HANDOVER_NATIVE')
    def test_history_alone_is_not_health_failure(self):
        from test_imu_fault_detail import ImuFaultDetail
        self.assertIsNone(ImuFaultDetail().reason(dict(fault='0',frozen='0',fault_events='1:active:thermal;2:cleared:thermal;')))
    def test_fault_change_logged_immediately_temperatures_at_one_hz(self):
        h=self.make();text='fault=0 frozen=0 mode=damping fault_event_seq=0 motor_temperatures=30,31'
        h.policy_feedback(text,10.);h.policy_feedback(text,10.1)
        self.assertEqual(len(h.events),1)
        h.policy_feedback(text.replace('fault=0','fault=1').replace('seq=0','seq=1'),10.2)
        self.assertEqual(len(h.events),2)
        h.policy_feedback(text.replace('seq=0','seq=2'),10.3)
        self.assertEqual(len(h.events),3)
        self.assertIn('motor_temperatures',h.events[-1][1]['policy'])
        h.policy_feedback(text.replace('seq=0','seq=2'),11.31);self.assertEqual(len(h.events),4)

if __name__=='__main__':unittest.main()
