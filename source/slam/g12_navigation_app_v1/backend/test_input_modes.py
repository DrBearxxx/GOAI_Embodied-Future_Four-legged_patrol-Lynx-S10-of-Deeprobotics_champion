"""Original manual axes vs physical navigation velocity; mocked robot transport."""
import socket,sys,threading,time,unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from core import MODES,NATIVE
from unified_control import Controller,Simulator,validate_axes
from zero_guardian import zero_packet,child
from wire import decode


class InputUnitsTests(unittest.TestCase):
    def test_manual_full_scale_does_not_inherit_navigation_speed_limit(self):
        a=Simulator();c=Controller(a)
        for seq,(action,axes) in enumerate((('select:basic',[0,0,0]),('stand',[0,0,0]),('run',[0,0,0]),('',[1,-1,1])),1):
            c.accept(dict(action=action,axes=axes,input_kind='native_axes',sample_seq=seq,sample_age_ms=0,fresh=True),10.)
        c.tick(10.)
        self.assertEqual(a.output,(1,-1,1));self.assertEqual(c.status(10.)['output_units'],'normalized')
    def test_physical_velocity_uses_policy_range_without_app_caps(self):
        a=Simulator();c=Controller(a)
        for seq,(action,axes) in enumerate((('select:basic',[0,0,0]),('stand',[0,0,0]),('run',[0,0,0]),('',[1.2,-.65,1.5])),1):
            c.accept(dict(action=action,axes=axes,input_kind='velocity',sample_seq=seq,sample_age_ms=0,fresh=True),10.)
        c.tick(10.)
        self.assertEqual(a.output,(1.2,-.65,1.5));self.assertEqual(c.status(10.)['output_units'],'m/s,m/s,rad/s')
    def test_nonfinite_and_out_of_range_axes_rejected(self):
        for values in ([1.01,0,0],[0,float('nan'),0],[0,0,float('inf')],[True,0,0]):
            with self.assertRaises(ValueError):validate_axes(values)
    def test_stop_zero_packet_matches_both_input_channels(self):
        for command in (0x100002,0x110002):
            packet=decode(zero_packet(3,command))
            self.assertEqual(packet['command'],command);self.assertFalse(any(packet['items'].values()))


@unittest.skipUnless(sys.platform=='linux','Mocked Linux adapter')
class HardwareInputTests(unittest.TestCase):
    def make(self):
        from test_hardware_contract import HardwareContract
        return HardwareContract().make()
    def tick(self,h,axes,now):
        h.node=None;h.last_hb=now;h.last_audit=now;h.last_send=now-1.
        h.rclpy=NS(spin_once=lambda *a,**k:None);h.read=lambda now:None
        h.guardian=NS(pulse=lambda *a:None)
        with patch('hardware.time.monotonic',return_value=now):h.tick(axes,now)
    def test_velocity_to_manual_only_switches_usage_mode_and_zeros_previous_channel(self):
        h=self.make();h.set_native_input_kind('native_axes',1.);h.select('basic',1.)
        self.assertEqual([s[1] for s in h.sent],[0x110002,0x500002])
        self.assertEqual(h.sent[-1][2],{'Mode':0});self.assertFalse(h.ready(1.))
        h.basic['ControlUsageMode']=0;h.basic_at=1.1;h.motion_at=1.1;h.advance_native(1.2)
        self.assertTrue(h.ready(1.2));self.assertEqual(h.native_command(),0x100002)
    def test_manual_to_navigation_only_switches_usage_mode(self):
        h=self.make();h.native_input_kind='native_axes';h.basic['ControlUsageMode']=0
        h.set_native_input_kind('velocity',1.);h.select('basic',1.)
        self.assertEqual([s[1] for s in h.sent],[0x100002,0x500002]);self.assertEqual(h.sent[-1][2],{'Mode':1})
        h.basic['ControlUsageMode']=1;h.basic_at=1.1;h.motion_at=1.1;h.advance_native(1.2)
        self.assertTrue(h.ready(1.2));self.assertEqual(h.native_command(),0x110002)
    def test_all_official_manual_policies_keep_full_scale_axes_on_wire(self):
        for policy in NATIVE:
            h=self.make();h.native_input_kind='native_axes';h.basic['ControlUsageMode']=0
            h.requested=h.confirmed=policy;h.motion.gait_state.gait=MODES[policy]
            self.tick(h,(1.,-.7,.9),1.)
            self.assertEqual(h.sent[-1],(0x100001,0x100002,dict(X=1.,Y=-.7,Yaw=.9,Z=0.,Roll=0.,Pitch=0.)))
    def test_manual_gait_change_keeps_original_mode_without_extra_mode_request(self):
        h=self.make();h.native_input_kind='native_axes';h.basic['ControlUsageMode']=0
        h.select('stairs',1.);h.motion.gait_state.gait=0x3003;h.motion_at=1.1;h.advance_native(1.2)
        self.assertTrue(h.ready(1.2));self.assertEqual(h.sent,[(0x100001,0x300002,{'GaitParam':0x3003})])
    def test_navigation_wire_preserves_physical_units(self):
        for policy in ('basic','stairs','platform'):
            h=self.make();h.requested=h.confirmed=policy;h.motion.gait_state.gait=MODES[policy]
            self.tick(h,(1.2,-.65,1.5),1.)
            self.assertEqual(h.sent[-1],(0x100001,0x110002,dict(X=1.2,Y=-.65,Yaw=1.5,Z=0.,Roll=0.,Pitch=0.)))
    def test_live_manual_rapid_retarget_keeps_enabled_but_pauses_output(self):
        h=self.make();h.native_input_kind='native_axes';h.basic['ControlUsageMode']=0
        c=Controller(h);c.native_input_kind='native_axes';c.enabled=True;c.axes=(1.,-.8,.7)
        for seq,policy in enumerate(('stairs','platform','basic'),1):
            now=1.+seq*.05
            c.accept(dict(action='select:'+policy,axes=[1.,-.8,.7],input_kind='native_axes',sample_seq=seq,sample_age_ms=0,fresh=True),now)
            self.assertTrue(c.enabled);self.assertTrue(h.native_input_ready(now))
            self.tick(h,tuple(c.status(now)['output']),now)
            self.assertEqual(h.sent[-1][1],0x100002);self.assertEqual(h.sent[-1][2]['X'],0.)
        self.assertEqual([s[2]['GaitParam'] for s in h.sent if s[1]==0x300002],[])
        self.assertEqual(c.policy_switch['policy'],'basic')
        self.assertFalse(any(s[1]==0x500002 for s in h.sent))
    def test_live_navigation_keeps_velocity_during_mode_reset_and_accepts_next_policy(self):
        h=self.make();h.select_live('stairs',1.)
        self.assertTrue(h.native_input_ready(1.))
        h.motion.gait_state.gait=0x3003;h.motion_at=1.1;h.basic['ControlUsageMode']=0
        self.tick(h,(.13,.04,-.1),1.2)
        self.assertEqual(h.phase,'WAIT_INPUT_MODE');self.assertTrue(h.native_input_ready(1.2))
        self.assertEqual(h.sent[-1][1],0x110002);self.assertEqual(h.sent[-1][2]['X'],.13)
        h.select_live('platform',1.25)
        self.assertEqual(h.sent[-1][2],dict(GaitParam=0x1002));self.assertTrue(h.native_input_ready(1.25))
        h.motion.gait_state.gait=0x1002;h.motion_at=1.3;self.tick(h,(.13,.04,-.1),1.4)
        h.basic['ControlUsageMode']=1;h.basic_at=h.motion_at=1.5;self.tick(h,(.13,.04,-.1),1.6)
        self.assertTrue(h.ready(1.6));self.assertEqual(h.sent[-1][2]['X'],.13)
    def test_live_navigation_mode_failure_stops_correctly_typed_output(self):
        h=self.make();h.select_live('stairs',1.)
        h.motion.gait_state.gait=0x3003;h.motion_at=1.1;h.basic['ControlUsageMode']=0
        self.tick(h,(.13,0,0),1.2);self.tick(h,(.13,0,0),6.3)
        self.assertEqual(h.phase,'FAILED');self.assertFalse(h.native_input_ready(6.3))
        self.assertFalse(any(h.sent[-1][2].values()))
    def test_live_switch_does_not_claim_ready_after_loss_of_native_control(self):
        for state in (0,100):
            h=self.make();h.select_live('stairs',1.);h.motion.motion_state.state=state
            self.assertFalse(h.native_input_ready(1.1));self.assertFalse(h.can_live_select('basic',1.1))
    def test_mode_switch_never_sends_nonzero_under_wrong_mode(self):
        h=self.make();h.set_native_input_kind('native_axes',1.);h.select('basic',1.)
        self.tick(h,(1,1,1),1.1)
        self.assertFalse(any(h.sent[-1][2].values()))
    def test_real_packet_encoder_keeps_full_manual_values(self):
        from hardware import Hardware
        h=Hardware.__new__(Hardware);packets=[]
        h.sock=NS(send=lambda packet:packets.append(packet));h.counter=0;h.tx={'axes':0,'nonzero_axes':0}
        h.send(0x100001,0x100002,dict(X=1.,Y=-1.,Yaw=.8,Z=0.,Roll=0.,Pitch=0.))
        result=decode(packets[0]);self.assertEqual(result['command'],0x100002)
        self.assertEqual(result['items']['X'],1.);self.assertEqual(result['items']['Y'],-1.)
        self.assertEqual(result['items']['Yaw'],.8)
    def test_wire_log_matches_successfully_encoded_velocity_and_source(self):
        from hardware import Hardware
        h=Hardware.__new__(Hardware);records=[];packets=[]
        h.sock=NS(send=lambda packet:packets.append(packet));h.counter=0;h.tx={'axes':0,'nonzero_axes':0}
        h.flight=NS(emit=lambda event,**fields:records.append(dict(event=event,**fields)))
        h.trace_context={'gateway_seq':12,'source':{'run_id':'nav-test','seq':34}}
        h.send(0x100001,0x110002,dict(X=1.2,Y=-.65,Yaw=1.5,Z=0.,Roll=0.,Pitch=0.))
        decoded=decode(packets[0]);log=records[-1]
        self.assertEqual(log['event'],'wire_tx');self.assertEqual(log['items'],decoded['items'])
        self.assertEqual(log['command'],'0x110002');self.assertEqual(log['context']['source']['seq'],34)
    def test_failed_socket_send_is_not_reported_as_transmitted(self):
        from hardware import Hardware
        def failed(packet):raise OSError('simulated send failure')
        h=Hardware.__new__(Hardware);h.sock=NS(send=failed);h.counter=0;records=[]
        h.flight=NS(emit=lambda event,**fields:records.append(event))
        with self.assertRaises(OSError):h.send(0x100001,0x110002,dict(X=.1,Y=0.,Yaw=0.,Z=0.,Roll=0.,Pitch=0.))
        self.assertEqual(records,['wire_error'])


@unittest.skipUnless(sys.platform=='linux','Guardian child uses Unix socket descriptors')
class GuardianChannelTests(unittest.TestCase):
    def check_channel(self,pulse,command):
        receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);receiver.bind(('127.0.0.1',0));receiver.settimeout(1.5)
        robot=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);robot.connect(receiver.getsockname())
        parent,worker=socket.socketpair();parent.settimeout(1.)
        thread=threading.Thread(target=child,args=(robot.detach(),worker.detach()),daemon=True);thread.start()
        try:
            parent.sendall(pulse);self.assertEqual(parent.recv(64),b'R')
            packet=decode(receiver.recv(65536));self.assertEqual(packet['command'],command)
            self.assertFalse(any(packet['items'].values()))
        finally:
            parent.close();thread.join(3.);receiver.close()
        self.assertFalse(thread.is_alive())
    def test_manual_watchdog_zeros_manual_channel(self):self.check_channel(b'J',0x100002)
    def test_navigation_watchdog_zeros_velocity_channel(self):self.check_channel(b'1',0x110002)


if __name__=='__main__':unittest.main()
