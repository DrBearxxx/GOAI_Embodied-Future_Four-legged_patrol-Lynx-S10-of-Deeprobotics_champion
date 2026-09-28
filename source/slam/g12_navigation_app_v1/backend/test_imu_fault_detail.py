"""Fault diagnostics use the node's cause, without constructing robot I/O."""
import sys
import unittest
from types import SimpleNamespace as NS


@unittest.skipUnless(sys.platform == 'linux', 'Linux adapter without robot I/O')
class ImuFaultDetail(unittest.TestCase):
    def reason(self, policy):
        from hardware import Hardware
        adapter = Hardware.__new__(Hardware)
        adapter.guardian = NS(healthy=lambda now: True)
        adapter.audit = {'ok': True}
        adapter.last_audit = adapter.motion_at = adapter.basic_at = adapter.policy_at = 10.
        adapter.basic = {'HES': 0, 'Charge': 0, 'Sleep': False}
        adapter.proc = NS(poll=lambda: None)
        adapter.policy = policy
        return adapter.health_reason(10.)

    def test_actual_cause_survives_frozen_flag(self):
        self.assertEqual(self.reason(dict(fault='1', frozen='1', fault_reason='IMU_receive_gap')),
                         'POLICY_FAULT:IMU_receive_gap')

    def test_explicit_freeze_is_not_an_unspecified_fault(self):
        self.assertEqual(self.reason(dict(fault='0', frozen='1')), 'POLICY_FROZEN')

    def test_valid_status_accepts_clock_diagnostics(self):
        self.assertIsNone(self.reason(dict(fault='0', frozen='0', body_clock='clock_jump')))

    def test_missing_status_has_distinct_error(self):
        self.assertEqual(self.reason({}), 'POLICY_STATUS_INCOMPLETE')
