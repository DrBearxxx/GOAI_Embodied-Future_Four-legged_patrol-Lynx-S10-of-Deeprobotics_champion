"""Shared runtime request/HTTP handler tested without starting ROS or hardware."""
import json
import threading
import time
import uuid
import urllib.request
import urllib.error
import unittest
from http.server import ThreadingHTTPServer
from runtime import MapService,handler
from mission import Mission
from protocol import Tickets,sign
from test_unified import FakeBridge,route,solution


class Fixture:
    request=MapService.request
    def __init__(self):
        self.lock=threading.RLock();self.tickets=Tickets();self.keepalive_tickets=Tickets(1)
        self.navigation_tickets=Tickets(.8);self.operator_tickets=Tickets(.4)
        self.bridge=FakeBridge();self.mission=Mission({'indoor':route()},self.bridge)
        self.navigator=self.mission.navigator;self.navigation_requests={};self.jobs={};self.map_id='fixture'
        self.state_time=time.monotonic();self.state={'solution':solution(t=self.state_time,x=0),'mission':self.mission.state()}


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.service=Fixture();self.key='fixture-api-key-'+'x'*40
        self.server=ThreadingHTTPServer(('127.0.0.1',0),handler(self.service,self.key));self.server.daemon_threads=True
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
    def tearDown(self):self.server.shutdown();self.server.server_close()
    def rpc(self,path,value,valid=True):
        value=dict(value,nonce=str(uuid.uuid4()));raw=json.dumps(value).encode()
        req=urllib.request.Request('http://127.0.0.1:'+str(self.server.server_port)+path,data=raw,
            headers={'X-S10-MAC':sign(self.key,path,raw) if valid else 'wrong'})
        with urllib.request.urlopen(req,timeout=1) as r:
            body=r.read();self.assertEqual(r.headers['X-S10-MAC'],sign(self.key,path,body))
            d=json.loads(body);self.assertEqual(d['nonce'],value['nonce']);return d
    def test_authentication_required(self):
        with self.assertRaises(urllib.error.HTTPError):self.rpc('/state',{},False)
    def test_teach_route_api_deduplicates_manual_marks_and_publishes_catalog(self):
        def action(kind,x=0,**extra):
            now=time.monotonic();self.service.state_time=now;self.service.state['solution']=solution(t=now,x=x)
            fresh=self.rpc('/state',{})
            body=dict(action='recording_'+kind,map_id='fixture',request_id=str(uuid.uuid4()),ticket=fresh['command_ticket'],**extra)
            result=self.rpc('/navigation',body);self.assertTrue(result['ok'],result);return body,result
        self.service.mission.owner='manual';self.service.mission.manual_policy='basic_normal'
        action('start',name='HTTP 录制')
        body,result=action('mark',2,name='入口')
        self.assertEqual(self.rpc('/navigation',body)['route_recording']['mark_count'],2)
        self.assertEqual(len(self.service.mission.recorder.marks),2)
        _,saved=action('save',5)
        state=self.rpc('/state',dict(include_catalog=True,recording_preview=True))
        self.assertIn(saved['saved_route'],state['mission']['catalog'])
        self.assertEqual(state['mission']['route_id'],'indoor');self.assertEqual(state['mission']['owner'],'manual')
        self.assertEqual(self.service.bridge.stops,0);self.assertTrue(state['route_recording']['trace'])
    def test_preset_official_only_over_http(self):
        fresh=self.rpc('/state',{})
        body=dict(action='set_policy',map_id='fixture',request_id=str(uuid.uuid4()),ticket=fresh['command_ticket'],route='indoor',segments=[0],policy='low')
        result=self.rpc('/navigation',body);self.assertFalse(result['ok']);self.assertIn('OFFICIAL',result['error'])
    def test_catalog_slim_poll_and_full_editor(self):
        self.assertNotIn('catalog',self.rpc('/state',{})['mission'])
        self.assertIn('catalog',self.rpc('/state',{'include_catalog':True})['mission'])
    def test_stop_does_not_need_expiring_ticket(self):
        self.service.mission.owner='manual'
        self.assertTrue(self.rpc('/stop',{})['ok']);self.assertEqual(self.service.mission.owner,'paused')
    def test_progress_applies_once_and_returns_target(self):
        fresh=self.rpc('/state',{});rid=str(uuid.uuid4())
        body=dict(action='set_progress',route='indoor',target_index=3,waypoint_id='3',map_id='fixture',request_id=rid,ticket=fresh['command_ticket'])
        r=self.rpc('/navigation',body);self.assertTrue(r['ok'],r);self.assertEqual(r['progress']['target_index'],3)
        self.assertEqual(self.rpc('/navigation',body)['progress'],r['progress'])
        self.assertEqual(self.service.bridge.stops,1);self.assertEqual(self.service.navigator.reached,[0,1,2])
    def test_manual_enable_before_rc_initializes_over_http(self):
        fresh=self.rpc('/state',{})
        result=self.rpc('/navigation',dict(action='manual',policy='basic',operator_session='new-app',
            map_id='fixture',request_id=str(uuid.uuid4()),ticket=fresh['command_ticket']))
        self.assertTrue(result['ok'],result);self.assertTrue(result['accepted'])
        self.assertEqual(self.service.mission.owner,'manual')
        self.assertIsNone(self.service.mission.rc)
    def test_replan_and_apply_no_automatic_start(self):
        now=time.monotonic();self.service.state_time=now;self.service.state['solution']=solution(t=now)
        fresh=self.rpc('/state',{})
        body=dict(action='replan',map_id='fixture',request_id=str(uuid.uuid4()),ticket=fresh['command_ticket'],goal=4)
        preview=self.rpc('/navigation',body);self.assertTrue(preview['ok'],preview)
        fresh=self.rpc('/state',{})
        body.update(action='apply_plan',request_id=str(uuid.uuid4()),ticket=fresh['command_ticket'],plan_id=preview['preview']['id'])
        applied=self.rpc('/navigation',body);self.assertTrue(applied['ok'],applied)
        self.assertEqual(self.service.mission.owner,'paused')
    def test_duplicate_action_not_replayed(self):
        fresh=self.rpc('/state',{})
        body=dict(action='set_policy',map_id='fixture',request_id=str(uuid.uuid4()),ticket=fresh['command_ticket'],route='indoor',segments=[0],policy='stairs_normal')
        self.assertTrue(self.rpc('/navigation',body)['ok']);self.assertTrue(self.rpc('/navigation',body)['ok'])
        self.assertEqual(self.service.mission.plans.revision,1)

    def test_calibration_draft_capture_and_apply_over_real_http(self):
        fresh=self.rpc('/state',{'calibration_route':'indoor'})
        self.assertEqual(fresh['calibration']['revision'],0)
        self.service.mission.owner='manual'
        body=dict(action='calibration_record',route='indoor',revision=0,point_id='1',map_id='fixture',request_id=str(uuid.uuid4()),ticket=fresh['command_ticket'])
        result=self.rpc('/navigation',body);self.assertTrue(result['ok'],result)
        self.assertEqual(self.service.mission.owner,'manual');self.assertEqual(self.service.bridge.stops,0)
        self.assertEqual(self.rpc('/navigation',body)['calibration'],result['calibration'])
        self.assertEqual(self.service.mission.plans.calibration.data['revision'],1)
        fresh=self.rpc('/state',{});body.update(action='calibration_apply',revision=1,request_id=str(uuid.uuid4()),ticket=fresh['command_ticket'])
        result=self.rpc('/navigation',body);self.assertTrue(result['ok'],result)
        self.assertEqual(self.service.mission.owner,'paused');self.assertFalse(self.service.navigator.active)
        self.assertEqual(self.service.navigator.route['waypoints'][1]['xyz'],[0.,0.,0.])


if __name__=='__main__':unittest.main()
