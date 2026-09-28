"""Synthetic graphs/telemetry only; never publishes into the robot domain."""
import copy
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav.dds_ownership import ANCHORS, COMMAND_TYPES, BARE, DdsOwnership, apply_audit, environment_reason

def endpoint(topic, part, entity=1):
    gid = bytes([1, 15, part])+bytes(9)+bytes([0, 0, entity, 3])
    return dict(gid=gid.hex(), name=BARE, namespace=BARE, type={**ANCHORS, **COMMAND_TYPES}[topic])

def graph():
    out = {key: [] for key in {**ANCHORS, **COMMAND_TYPES}}
    for topic in ('/MOTION_INFO', '/JOINTS_DATA', '/IMU_DATA', '/JOINTS_CMD'):
        out[topic] = [endpoint(topic, 1)]
    for topic in ('/MOTION_STATE', '/NAV_STATUS', '/GAIT'): out[topic] = [endpoint(topic, 2)]
    out['/OOA_STATUS'] = [endpoint('/OOA_STATUS', 3)]
    out['/PLANNER_STATUS'] = [endpoint('/PLANNER_STATUS', 4)]
    out['/NAV_CMD'] = [endpoint('/NAV_CMD', 3), endpoint('/NAV_CMD', 4)]
    return out

class OwnershipTests(unittest.TestCase):
    def ready(self):
        d, g = DdsOwnership(), graph()
        d.telemetry('/MOTION_INFO', bytes.fromhex(g['/MOTION_INFO'][0]['gid']), 1.)
        d.audit(g, 1.)
        d.telemetry('/MOTION_INFO', bytes.fromhex(g['/MOTION_INFO'][0]['gid']), 2.1)
        a = d.audit(g, 2.1)
        self.assertTrue(a['ok'], a)
        return d, g

    def test_native_roles_are_not_counted_as_foreign(self):
        d, g = self.ready()
        a = d.audit(g, 2.2)
        self.assertEqual(len(a['native']), 4)
        env = apply_audit(dict(mono=2.2, rl_service_active=False, localization_publishers=1), a)
        self.assertIsNone(environment_reason(env, 2.2))
        self.assertEqual(env['other_nav_publishers'], 0)
        self.assertEqual(env['mode_publishers'], 0)

    def test_bare_name_never_suffices(self):
        d, g = self.ready()
        for topic in COMMAND_TYPES:
            changed = copy.deepcopy(g)
            changed[topic].append(endpoint(topic, 99))
            a = d.audit(changed, 2.2)
            self.assertFalse(a['ok'])
            self.assertTrue(a['unknown'])

    def test_actual_nav_even_zero_or_unknown_source_is_conflict(self):
        d, g = self.ready()
        d.nav_received(bytes(16), 2.2)
        a = d.audit(g, 2.21)
        self.assertEqual(a['reason'], 'COMPETING_NAV_TRAFFIC')
        self.assertEqual(a['nav_messages'], 1)
        d.telemetry('/MOTION_INFO', bytes.fromhex(g['/MOTION_INFO'][0]['gid']), 3.3)
        self.assertTrue(d.audit(g, 3.3)['ok'])

    def test_motion_requires_live_matching_telemetry(self):
        d, g = self.ready()
        self.assertFalse(d.audit(g, 3.5)['ok'])
        d.telemetry('/MOTION_INFO', bytes(16), 3.5)
        self.assertFalse(d.audit(g, 3.5)['ok'])
        d.telemetry('/MOTION_INFO', bytes.fromhex(g['/MOTION_INFO'][0]['gid']), 3.5, valid=False)
        self.assertFalse(d.audit(g, 3.5)['ok'])

    def test_mixed_telemetry_participants_cannot_whitelist(self):
        d, g = self.ready()
        g['/IMU_DATA'].append(endpoint('/IMU_DATA', 99))
        self.assertFalse(d.audit(g, 2.2)['ok'])

    def test_unknown_rmw_type_namespace_and_zero_gid_block(self):
        for field, value in (('name', 'fake'), ('namespace', '/'), ('gid', bytes(16).hex()), ('type', 'bad')):
            d, g = self.ready()
            g['/JOINTS_CMD'][0][field] = value
            self.assertFalse(d.audit(g, 2.2)['ok'])
        d, g = self.ready()
        self.assertFalse(d.audit(g, 2.2, 'unknown_rmw')['ok'])

    def test_handler_active_remains_blocked_until_idle(self):
        d, g = self.ready()
        gid = bytes.fromhex(g['/OOA_STATUS'][0]['gid'])
        d.telemetry('/OOA_STATUS', gid, 2.2, idle=False)
        self.assertEqual(d.audit(g, 2.3)['reason'], 'FACTORY_HANDLER_NOT_IDLE')
        d.telemetry('/OOA_STATUS', gid, 2.3, idle=True)
        self.assertTrue(d.audit(g, 2.3)['ok'])

    def test_graph_revision_and_stable_window(self):
        d, g = self.ready()
        old = d.revision
        g['/NAV_CMD'].pop()
        a = d.audit(g, 2.2)
        self.assertEqual(a['reason'], 'WAIT_DDS_STABLE_1S')
        self.assertGreater(a['revision'], old)

    def test_process_and_stale_environment_still_block(self):
        d, g = self.ready()
        env = apply_audit(dict(mono=2.2, rl_service_active=True), d.audit(g, 2.2))
        self.assertEqual(environment_reason(env, 2.2), 'COMPETING_CONTROLLER_PROCESS')
        self.assertEqual(environment_reason(env, 3.), 'OWNERSHIP_CHECK_STALE')

    def test_empty_graph_is_not_proof_of_no_competition(self):
        d = DdsOwnership()
        d.audit({}, 1.)
        self.assertEqual(d.audit({}, 3.)['reason'], 'DDS_TELEMETRY_UNAVAILABLE')

if __name__ == '__main__': unittest.main(verbosity=2)
