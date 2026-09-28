"""Protocol and unit-preserving checks; no robot transport is constructed."""
import sys,unittest
from core import MODES,NATIVE,WAYPOINT
from unified_control import Controller,Simulator


class StepMoveGatewayTests(unittest.TestCase):
    def test_unsigned_factory_code_belongs_to_native_gaits(self):
        self.assertEqual(MODES['step_move'],0xF002)
        self.assertIn('step_move',NATIVE);self.assertNotIn('step_move',WAYPOINT)

    def test_manual_axes_and_navigation_velocity_are_not_rescaled(self):
        for kind,values in (('native_axes',[1.,-.8,.9]),('velocity',[3.2,-.7,1.4])):
            a=Simulator();c=Controller(a)
            for seq,(action,axes) in enumerate((('select:step_move',[0,0,0]),('stand',[0,0,0]),('run',[0,0,0]),('',values)),1):
                c.accept(dict(action=action,axes=axes,input_kind=kind,sample_seq=seq,sample_age_ms=0,fresh=True),10.)
            c.tick(10.)
            self.assertEqual(a.output,tuple(values));self.assertEqual(c.status(10.)['requested'],'step_move')
            self.assertEqual(c.status(10.)['input_kind'],kind)

    @unittest.skipUnless(sys.platform=='linux','Mocked Linux adapter')
    def test_gait_request_and_actual_feedback_in_both_input_modes(self):
        from test_hardware_contract import HardwareContract
        for kind,mode in (('native_axes',0),('velocity',1)):
            h=HardwareContract().make();h.native_input_kind=kind;h.basic['ControlUsageMode']=mode
            h.select('step_move',1.)
            self.assertEqual(h.sent,[(0x100001,0x300002,{'GaitParam':61442})])
            h.motion.gait_state.gait=0xF002;h.motion_at=1.1;h.advance_native(1.2)
            if mode==1:
                self.assertEqual(h.sent[-1],(0x100002,0x500002,{'Mode':1}))
                h.basic_at=h.motion_at=1.3;h.advance_native(1.4)
            self.assertTrue(h.ready(1.4));self.assertEqual(h.status(1.4)['actual_policy'],'step_move')
            self.assertTrue(h.can_live_select('platform',1.4))

    @unittest.skipUnless(sys.platform=='linux','Mocked Linux adapter')
    def test_switch_from_platform_uses_existing_pause_and_resumes_without_new_enable(self):
        from test_hardware_contract import HardwareContract
        h=HardwareContract().make(gait=0x1002);h.confirmed=h.requested='platform'
        outputs=[];h.tick=lambda values,t:outputs.append(tuple(values))
        c=Controller(h);c.enabled=True
        for seq,t in enumerate((1.,1.2,1.4,1.51),1):
            c.accept(dict(action='select:step_move' if seq==1 else '',axes=[2.3,.1,.2],
                input_kind='velocity',sample_seq=seq,sample_age_ms=0,fresh=True),t)
            c.tick(t)
        self.assertEqual(h.sent,[(0x100001,0x300002,{'GaitParam':0xF002})])
        self.assertTrue(all(not any(v) for v in outputs))
        h.motion.gait_state.gait=0xF002;h.motion_at=1.6;h.advance_native(1.6)
        h.basic_at=h.motion_at=1.7;h.advance_native(1.8)
        c.accept(dict(action='',axes=[2.3,.1,.2],input_kind='velocity',sample_seq=5,sample_age_ms=0,fresh=True),1.8)
        c.tick(1.8)
        self.assertTrue(c.enabled);self.assertEqual(outputs[-1],(2.3,.1,.2));self.assertIsNone(c.policy_switch)


if __name__=='__main__':unittest.main()
