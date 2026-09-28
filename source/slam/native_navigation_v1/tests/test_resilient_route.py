"""Synthetic faults only. No robot sockets, commands or hardware acceptance."""
import copy
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from native_nav import bootstrap
from native_nav.resilient_route import ResilientRoute
from native_nav.recovery import RecoveryWindow
from native_nav.reconnecting_client import ReconnectingClient
from native_nav.route_checkpoint import RouteCheckpoint
from native_nav.ingress import ResilientIngress
from test_route import data,simple_route


def sample(t,p=(0.,0.,0.,0.),generation=None,**kwargs):
    s,f,e=data(t,p,**kwargs)
    f['feedback']=dict(speed=0.,wz=0.)
    if generation is not None:s['generation']=s['solution']['generation']=list(generation)
    return s,f,e


class ResilientRouteTests(unittest.TestCase):
    def ready(self,route=None,p=(0.,0.,0.,0.)):
        g=ResilientRoute(route or simple_route(),'map1','boot1')
        for i in range(61):c=g.tick(i*.04,*sample(i*.04,p))
        self.assertTrue(c['ready'],c)
        self.assertFalse(g.armed)
        return g

    def active(self):
        g=self.ready();self.assertTrue(g.arm(2.4)[0])
        g.tick(2.44,*sample(2.44));c=g.tick(2.48,*sample(2.48))
        self.assertTrue(g.armed,c);self.assertGreater(c['vx'],0)
        return g

    def recover(self,g,start=2.56,count=65,**kwargs):
        return [g.tick(start+i*.04,*sample(start+i*.04,**kwargs)) for i in range(count)]

    def test_counter_correction_is_not_new_permission(self):
        g=self.active()
        for i in range(300):
            t=2.52+i*.04;c=g.tick(t,*sample(t,generation=[1,i+1]))
            self.assertTrue(g.armed,c);self.assertFalse(g.holding,c)
        self.assertEqual(g.reassociation_notices,300)
        self.assertEqual(g.target,1)

    def test_long_perception_faults_hold_and_recover(self):
        faults=[dict(age=.56),dict(imu_age=.09),dict(safety_age=.41),dict(mode='LOST')]
        for fault in faults:
            g=self.active();c=g.tick(2.52,*sample(2.52,**fault))
            self.assertTrue(g.armed,c);self.assertTrue(g.holding,c);self.assertEqual(c['vx'],0)
            commands=self.recover(g)
            self.assertTrue(all(c['vx']==0 for c in commands[:50]))
            self.assertTrue(any(c['state']=='RESUME_READY' for c in commands))
            self.assertTrue(g.armed);self.assertEqual(g.resume_count,1);self.assertEqual(g.target,1)
            first=next(c for c in commands if c['vx']>0)
            self.assertLessEqual(first['vx'],.0041)

    def test_long_total_loss_keeps_process_mission_and_zero(self):
        g=self.active()
        for i in range(3000):
            t=2.52+i*.04;s,f,e=sample(t);c=g.tick(t,None,f,e)
            self.assertEqual(c['vx'],0);self.assertTrue(g.holding);self.assertTrue(g.armed)
        self.assertEqual(g.reached,[0]);self.assertEqual(g.target,1)
        self.recover(g,start=t+.04)
        self.assertEqual(g.resume_count,1)

    def test_soft_packet_corruption_and_time_regression_are_quarantined(self):
        changes=[lambda s:s.update(seq=-1),lambda s:s.update(pose=[float('nan')]*4),
                 lambda s:s.update(mono=1.,seq=1),lambda s:s['solution'].update(velocity_body=[99,0,0])]
        for change in changes:
            g=self.active();s,f,e=sample(2.52);change(s);c=g.tick(2.52,s,f,e)
            self.assertTrue(g.holding,c);self.assertTrue(g.armed,c);self.assertEqual(c['vx'],0)
            self.recover(g);self.assertEqual(g.resume_count,1)

    def test_real_jump_does_not_pass_by_waiting_or_counter_change(self):
        for p in ((.5,0,0,0),(0,0,0,.8)):
            g=self.active()
            for i in range(150):
                t=2.52+i*.04;c=g.tick(t,*sample(t,p,generation=[1,i+1]))
                self.assertEqual(c['vx'],0);self.assertEqual(c['wz'],0);self.assertTrue(g.holding)
            self.assertEqual(g.resume_count,0);self.assertEqual(g.target,1)

    def test_new_map_epoch_requires_stationary_window(self):
        g=self.active();c=g.tick(2.52,*sample(2.52,generation=[2,0]))
        self.assertTrue(g.holding);self.assertEqual(c['vx'],0)
        self.recover(g,generation=[2,0]);self.assertEqual(g.resume_count,1)

    def test_blocked_rear_waits_and_recovers(self):
        g=self.active()
        for i in range(30):
            t=2.52+i*.04;s,f,e=sample(t);s['safety_records']['rear']['blocked']=True
            c=g.tick(t,s,f,e);self.assertTrue(g.holding);self.assertEqual(c['vx'],0)
        self.recover(g,start=t+.04);self.assertEqual(g.resume_count,1)

    def test_recovery_requires_real_stationary_feedback(self):
        for speed in (None,.04):
            g=self.active();g.tick(2.52,*sample(2.52,mode='LOST'))
            for i in range(100):
                t=2.56+i*.04;s,f,e=sample(t)
                f['feedback']={} if speed is None else dict(speed=speed,wz=0)
                c=g.tick(t,s,f,e);self.assertTrue(g.holding);self.assertEqual(c['vx'],0)
            self.assertEqual(g.resume_count,0)

    def test_hard_authority_identity_and_stall_never_auto_arm(self):
        changes=[lambda s,f,e:f.update(armed=False),lambda s,f,e:f.update(session='new'),
                 lambda s,f,e:f.update(ready=False),lambda s,f,e:f.update(backend='simulation'),
                 lambda s,f,e:e.update(other_nav_publishers=1),lambda s,f,e:s.update(run_id='new'),
                 lambda s,f,e:s.update(asset_id='wrong'),lambda s,f,e:s.update(boot_id='wrong')]
        for change in changes:
            g=self.active();args=sample(2.52);change(*args);c=g.tick(2.52,*args)
            self.assertFalse(g.armed,c);self.assertEqual(c['vx'],0)
            self.assertTrue(all(c['vx']==0 for c in self.recover(g)))
            self.assertFalse(g.armed);self.assertEqual(g.target,1)
        g=self.active();g.tick(2.9,*sample(2.9));self.assertFalse(g.armed)

    def test_operator_stop_during_hold_never_auto_resumes(self):
        g=self.active();g.tick(2.52,*sample(2.52,mode='LOST'));g.stop('OPERATOR_STOP')
        self.assertTrue(all(c['vx']==0 for c in self.recover(g)))
        self.assertFalse(g.armed);self.assertEqual(g.resume_count,0)

    def test_window_allows_normal_intermeasurement_degradation(self):
        g=ResilientRoute(simple_route(),'map1','boot1')
        for i in range(140):
            t=i*.04;measurement=int(t/.36)*.36-.02
            s,f,e=sample(t,age=t-measurement,mode='TRACKING' if t-measurement<.25 else 'DEGRADED')
            c=g.tick(t,s,f,e)
        self.assertTrue(c['ready'],c);self.assertFalse(g.armed)

    def test_window_repeated_and_unstable_measurements_rejected(self):
        for unstable in (False,True):
            w=RecoveryWindow()
            for i in range(100):
                t=i*.04;s=sample(t)[0]
                if unstable:s['solution']['measured_pose'][0]=.3 if i%2 else 0.
                else:s['solution']['measurement_mono']=0.
                self.assertFalse(w.observe(t,s,True,True))

    def test_full_synthetic_indoor_route_with_faults(self):
        route=json.loads((bootstrap.INDOOR/'assets/route.json').read_text());p=np.r_[route['waypoints'][0]['xyz'],0.]
        g=self.ready(route,p);self.assertTrue(g.arm(2.4)[0])
        for i in range(8000):
            t=2.44+i*.04;fault=100<=i%650<120
            c=g.tick(t,*sample(t,p,generation=[1,i//19],mode='LOST' if fault else 'TRACKING'))
            self.assertTrue(g.armed or g.complete,c)
            if fault:self.assertEqual(c['vx'],0)
            if g.complete:break
            self.assertLessEqual(c['vx'],.1);self.assertLessEqual(abs(c['wz']),.2)
            p[0]+=c['vx']*np.cos(p[3])*.04;p[1]+=c['vx']*np.sin(p[3])*.04;p[3]+=c['wz']*.04
            k=max(1,g.target);a,b=g.xyz[k-1:k+1];v=b-a;u=np.clip((p[:3]-a)@v/max(v@v,1e-9),0,1);p[2]=(a+u*v)[2]
        self.assertTrue(g.complete);self.assertEqual(g.reached,list(range(len(route['waypoints']))))
        self.assertGreater(g.resume_count,3)
        print(json.dumps(dict(test='synthetic_fault_route_NOT_physical',seconds=t,resumes=g.resume_count,points=len(g.reached))))


class PersistenceAndIngressTests(unittest.TestCase):
    def test_progress_restore_never_restores_arm(self):
        with tempfile.TemporaryDirectory() as folder:
            r=simple_route();g=ResilientRoute(r,'map1','boot1');g.target=1;g.reached=[0];g.armed=True
            cp=RouteCheckpoint(Path(folder)/'progress.json',r,'map1');self.assertTrue(cp.save(g))
            h=ResilientRoute(r,'map1','boot1');cp.restore(h)
            self.assertEqual(h.target,1);self.assertFalse(h.armed)
            with self.assertRaises(ValueError):RouteCheckpoint(cp.path,r,'other-map').restore(h)
            state=json.loads(cp.path.read_text());state['reached']=[False];cp.path.write_text(json.dumps(state))
            with self.assertRaises(ValueError):cp.restore(h)

    def test_checkpoint_disk_error_is_not_process_exit(self):
        cp=RouteCheckpoint('/unused/path',simple_route(),'map1');g=ResilientRoute(simple_route(),'map1','boot1')
        with patch.object(Path,'mkdir',side_effect=OSError('disk full')):self.assertFalse(cp.save(g))
        self.assertIsNotNone(cp.error)

    def test_bad_packet_does_not_restamp_good_data(self):
        i=ResilientIngress();info=dict(source_timestamp=1_000_000_000,received_timestamp=1_000_000_000)
        self.assertTrue(i.receive(json.dumps(dict(mono=1.)),info,1_000_000_000,1.,b'writer1'))
        self.assertFalse(i.receive('bad',info,1_050_000_000,1.05,b'writer1'))
        self.assertEqual(i.get(1.1)['mono'],1.);self.assertEqual(i.received,1.)
        self.assertIsNone(i.get(1.26));self.assertEqual(i.dropped,1)
        info=dict(source_timestamp=1_100_000_000,received_timestamp=1_100_000_000)
        self.assertFalse(i.receive('{"mono":1.1}',info,1_100_000_000,1.1,b'writer2'))
        self.assertEqual(i.hard_fault,'DDS_WRITER_CHANGED');self.assertIsNone(i.value)

    def test_nonfinite_json_does_not_crash_strict_route_log(self):
        i=ResilientIngress();info=dict(source_timestamp=1_000_000_000,received_timestamp=1_000_000_000)
        self.assertFalse(i.receive('{"mono":1.0,"pose":[NaN]}',info,1_000_000_000,1.,b'writer'))
        self.assertEqual(i.error,'NONFINITE_JSON');self.assertIsNone(i.get(1.))
        json.dumps(dict(snapshot=i.get(1.)),allow_nan=False)

    def test_tcp_hold_keeps_existing_lease_without_another_arm(self):
        server=subprocess.Popen([sys.executable,'-m','native_nav.gateway','--simulation','--port','0'],
            cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        client=None
        try:
            port=json.loads(server.stdout.readline())['port'];client=ReconnectingClient(port)
            g=ResilientRoute(simple_route(),'map1','boot1');start=time.monotonic()
            pending=False;active=None;arms=0;held=0;resumed=False
            while time.monotonic()-start<7.:
                now=time.monotonic();t=now-start;client.poll(now);st=client.get(now)
                s,f,e=sample(t)
                # Simulation-only backend translation; never used in route_ros.
                f=dict(st,mono=client.received-start,backend='execute') if st else None
                if active is not None and .25<t-active<.75:s['solution']['mode']='LOST'
                c=g.tick(t,s,f,e)
                if active is None and not pending and c.get('ready') and client.action('ARM',now):
                    pending=True;arms+=1
                if pending and st and st['armed']:
                    self.assertTrue(g.arm(t)[0]);active=t;pending=False
                if g.armed:
                    client.action('VELOCITY',now,[c['vx'],c['vy'],c['wz']])
                    if g.holding:
                        held+=1;self.assertEqual(c['vx'],0);self.assertTrue(st['armed'])
                    if g.resume_count and c['vx']>0:resumed=True;break
                time.sleep(.04)
            self.assertEqual(arms,1);self.assertGreater(held,40);self.assertTrue(resumed)
        finally:
            if client:client.close()
            server.terminate();server.communicate(timeout=3)

    def test_nonblocking_reconnect_no_action_replay(self):
        listener=socket.socket();listener.bind(('127.0.0.1',0));port=listener.getsockname()[1];listener.close()
        client=ReconnectingClient(port);server=None
        try:
            start=time.monotonic()
            while time.monotonic()-start<.35:client.poll(time.monotonic());time.sleep(.01)
            self.assertFalse(client.closed);self.assertGreater(client.disconnections,0)
            self.assertFalse(client.action('ARM',time.monotonic()))
            server=subprocess.Popen([sys.executable,'-m','native_nav.gateway','--simulation','--port',str(port)],
                cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            self.assertEqual(json.loads(server.stdout.readline())['port'],port)
            start=time.monotonic();sessions=set()
            while time.monotonic()-start<4:
                now=time.monotonic();client.poll(now);s=client.get(now)
                if s:
                    self.assertFalse(s['armed']);self.assertEqual(s['sent_nonzero'],0);sessions.add(s['session'])
                    if len(sessions)==1:client.current.close()
                    if len(sessions)>1:break
                time.sleep(.01)
            self.assertGreaterEqual(len(sessions),2);self.assertFalse(client.closed)
        finally:
            client.close()
            if server:
                server.terminate();server.communicate(timeout=3)


if __name__=='__main__':unittest.main(verbosity=2)
