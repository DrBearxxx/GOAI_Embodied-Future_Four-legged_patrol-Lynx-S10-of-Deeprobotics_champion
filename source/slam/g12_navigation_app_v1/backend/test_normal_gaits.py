"""Ordinary factory gaits share native joystick and pause/selection behavior."""
import sys
import unittest
from core import MODES
from unified_control import Controller, Simulator


class NormalControllerTests(unittest.TestCase):
    def test_full_scale_manual_units_for_both_normal_gaits(self):
        for policy in ('basic_normal', 'stairs_normal'):
            with self.subTest(policy=policy):
                adapter = Simulator(); control = Controller(adapter)
                for seq, (action, axes) in enumerate((('select:'+policy, [0,0,0]),
                    ('stand', [0,0,0]), ('run', [0,0,0]), ('', [1,-1,.9])), 1):
                    control.accept(dict(action=action, axes=axes, input_kind='native_axes',
                        sample_seq=seq, sample_age_ms=0, fresh=True), 10.)
                control.tick(10.)
                self.assertEqual(adapter.output, (1,-1,.9))
                self.assertEqual(control.status(10.)['input_kind'], 'native_axes')


@unittest.skipUnless(sys.platform == 'linux', 'Mocked Linux hardware without transport')
class NormalHardwareTests(unittest.TestCase):
    def test_factory_gait_ids_and_feedback(self):
        from test_hardware_contract import HardwareContract
        for policy, gait in (('basic_normal', 0x1001), ('stairs_normal', 0x1003)):
            with self.subTest(policy=policy):
                h = HardwareContract().make()
                h.native_input_kind='native_axes'; h.basic['ControlUsageMode']=0
                h.select(policy, 1.)
                self.assertEqual(h.sent, [(0x100001, 0x300002, {'GaitParam':gait})])
                h.motion.gait_state.gait=gait; h.motion_at=1.1
                h.advance_native(1.2)
                self.assertTrue(h.ready(1.2))
                self.assertEqual(h.status(1.2)['actual_policy'], policy)
                self.assertTrue(h.can_live_select('stairs', 1.2))

    def test_normal_switch_pauses_before_native_request(self):
        from test_hardware_contract import HardwareContract
        h=HardwareContract().make(); h.native_input_kind='native_axes'; h.basic['ControlUsageMode']=0
        outputs=[]; h.tick=lambda axes, now: outputs.append(tuple(axes))
        control=Controller(h); control.enabled=True; control.native_input_kind='native_axes'
        for seq,t in enumerate((1.,1.2,1.4,1.51),1):
            control.accept(dict(action='select:stairs_normal' if seq==1 else '',
                axes=[1,0,.7],input_kind='native_axes',sample_seq=seq,sample_age_ms=0,fresh=True),t)
            control.tick(t)
            if t<1.5:self.assertFalse(h.sent)
        self.assertEqual(h.sent, [(0x100001,0x300002,{'GaitParam':0x1003})])
        self.assertTrue(all(not any(values) for values in outputs))
