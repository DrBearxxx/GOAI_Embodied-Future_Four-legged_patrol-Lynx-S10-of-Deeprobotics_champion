"""Real localhost HTTP, tickets and velocity/stop watchdogs; no hardware."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from gateway_bridge import GatewayBridge
sys.path.insert(0,str(Path(__file__).parent/'backend'))
from server import Server
from unified_control import Controller,Simulator


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.key='test-only-'+'x'*48
        self.adapter=Simulator();self.server=Server(('127.0.0.1',0),self.key.encode(),Controller(self.adapter))
        self.threads=[threading.Thread(target=self.server.tick,daemon=True),threading.Thread(target=self.server.serve_forever,daemon=True)]
        for t in self.threads:t.start()
        self.bridge=GatewayBridge('http://127.0.0.1:'+str(self.server.server_port),self.key)
    def tearDown(self):
        self.bridge.close();self.server.running=False;self.server.shutdown();self.server.server_close()
    def wait(self,predicate,timeout=2,produce=False):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if produce:self.bridge.send(dict(active=True,execution_requested=True,vx=0,vy=0,wz=0),time.monotonic())
            if predicate():return
            time.sleep(.02)
        self.fail('timeout: '+repr(self.bridge.state()))
    def action(self,name):self.bridge.action(name,name,time.monotonic())
    def test_full_policy_velocity_stop_roundtrip(self):
        class Trace:
            def __init__(self):self.records=[]
            def emit(self,event,**fields):self.records.append(dict(event=event,**fields))
        trace=Trace();self.bridge.trace=trace
        self.wait(lambda:self.bridge.state().get('backend')=='simulation')
        self.action('select:stairs');self.wait(lambda:self.adapter.selected=='stairs')
        self.action('stand');self.wait(lambda:self.bridge.policy_ready('stairs'))
        self.action('run');self.wait(self.bridge.ready,produce=True)
        end=time.monotonic()+.3
        while time.monotonic()<end:
            self.bridge.send(dict(active=True,execution_requested=True,vx=1.2,vy=-.65,wz=1.5,run_id='trace-test',seq=42),time.monotonic());time.sleep(.03)
        self.assertEqual(self.adapter.output,(1.2,-.65,1.5))
        requests=[r for r in trace.records if r['event']=='gateway_request' and r['source'].get('run_id')=='trace-test']
        self.assertTrue(requests);self.assertEqual(requests[-1]['source']['seq'],42)
        self.assertTrue(any(r['event']=='gateway_reply' and r['gateway_seq']==requests[-1]['gateway_seq'] for r in trace.records))
        self.assertNotIn(self.key,json.dumps(trace.records));self.assertNotIn('ticket',json.dumps(trace.records))
        self.bridge.stop();self.wait(lambda:not self.server.control.enabled)
        self.assertEqual(self.server.control.axes,(0.,0.,0.))
    def test_producer_loss_disarms_even_with_network_alive(self):
        self.wait(lambda:self.bridge.state().get('backend')=='simulation')
        for action in ('select:basic','stand'):
            self.action(action);time.sleep(.13)
        self.action('run');self.wait(self.bridge.ready,produce=True)
        self.wait(lambda:not self.server.control.enabled,timeout=1.5)
    def test_manual_full_scale_survives_http_gateway_without_velocity_scaling(self):
        self.wait(lambda:self.bridge.state().get('backend')=='simulation')
        self.bridge.action('select:basic','manual-select',time.monotonic(),input_kind='native_axes')
        self.wait(lambda:self.adapter.selected=='basic');self.action('stand')
        self.wait(lambda:self.bridge.policy_ready('basic','native_axes'))
        end=time.monotonic()+1.5;run_sent=False
        while time.monotonic()<end:
            self.bridge.send(dict(active=True,execution_requested=True,input_kind='native_axes',axes=[1.,-.8,.7]),time.monotonic())
            if not run_sent:
                self.bridge.action('run','manual-run',time.monotonic(),input_kind='native_axes');run_sent=True
            if self.adapter.output==(1.,-.8,.7):break
            time.sleep(.02)
        self.assertEqual(self.adapter.output,(1.,-.8,.7));self.assertEqual(self.bridge.state()['input_kind'],'native_axes')
        self.assertFalse(self.bridge.policy_ready('basic','velocity'))
        self.bridge.stop();self.wait(lambda:not self.server.control.enabled)
    def test_tickets_cannot_replay(self):
        self.bridge.close();time.sleep(.1)
        nonce='ticket-test';offer=self.bridge._rpc('/ticket',dict(nonce=nonce,session='isolated'))
        request=dict(ticket=offer['ticket'],action='select:basic',axes=[0,0,0],input_kind='velocity',fresh=True,sample_seq=1,sample_age_ms=0)
        self.bridge._rpc('/control',request)
        import urllib.error
        with self.assertRaises(urllib.error.HTTPError):self.bridge._rpc('/control',request)

    def test_live_switch_roundtrip_pauses_then_resumes_without_another_run(self):
        # Same-controller capabilities, with instantaneous simulated feedback.
        self.adapter.can_live_select=lambda mode,now:self.adapter.ready(now)
        self.adapter.select_live=self.adapter.select
        original_status=self.adapter.status
        self.adapter.status=lambda now:dict(original_status(now),live_input_ready=self.adapter.ready(now))
        self.wait(lambda:self.bridge.state().get('backend')=='simulation')
        self.action('select:basic');self.wait(lambda:self.adapter.selected=='basic')
        self.action('stand');self.wait(lambda:self.bridge.policy_ready('basic'))
        self.action('run');self.wait(self.bridge.ready,produce=True)
        self.action('select:stairs');rows=[];end=time.monotonic()+1.2
        while time.monotonic()<end:
            self.bridge.send(dict(active=True,execution_requested=True,vx=1.2,vy=-.65,wz=1.5),time.monotonic())
            s=self.bridge.state()
            rows.append((time.monotonic(),s.get('policy_switch_paused'),s.get('enabled'),s.get('output'),self.adapter.selected))
            time.sleep(.02)
        paused=[r for r in rows if r[1]]
        self.assertGreaterEqual(len(paused),15)
        self.assertGreater(paused[-1][0]-paused[0][0],.4)
        self.assertTrue(all(r[2] and r[3]==[0.,0.,0.] for r in paused))
        self.assertEqual(self.adapter.selected,'stairs');self.assertTrue(self.bridge.ready())
        self.assertEqual(self.adapter.output,(1.2,-.65,1.5))


class LiveFeedbackTests(unittest.TestCase):
    def test_pending_gait_feedback_does_not_interrupt_enabled_stream(self):
        b=GatewayBridge();b.received=time.monotonic()
        b.value=dict(ready=False,live_input_ready=True,enabled=True,requested='stairs',confirmed='none',input_kind='velocity')
        self.assertTrue(b.ready());self.assertFalse(b.policy_ready('stairs','velocity'))
        b.value['enabled']=False;self.assertFalse(b.ready())
        b.value['enabled']=True;b.received-=1.;self.assertFalse(b.ready())


if __name__=='__main__':unittest.main()
