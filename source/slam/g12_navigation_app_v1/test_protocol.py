import copy
import math
import unittest
import uuid
from protocol import seed_request, Tickets, sign

class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.body=dict(request_id=str(uuid.uuid4()),map_id='map',xyz=[0,0,0],radius=3)
        self.bounds=[[-5,-5,-1],[5,5,5]]
    def check(self):return seed_request(self.body,'map',self.bounds)
    def test_valid_scope_not_pose(self):
        v=self.check();self.assertNotIn('pose',v);self.assertEqual(v['height_tolerance'],.8)
    def test_wrong_map(self):
        self.body['map_id']='old'
        with self.assertRaisesRegex(ValueError,'MAP_MISMATCH'):self.check()
    def test_nonfinite(self):
        for bad in (math.nan,math.inf,-math.inf,True,'1',None):
            self.body['xyz'][0]=bad
            with self.assertRaises(ValueError):self.check()
    def test_radius_limits(self):
        for bad in (0, .9,5.1,100,True,math.nan):
            self.body['radius']=bad
            with self.assertRaises(ValueError):self.check()
    def test_radius_endpoints(self):
        for v in (1,5):self.body['radius']=v;self.check()
    def test_outside(self):
        self.body['xyz'][2]=7
        with self.assertRaisesRegex(ValueError,'OUTSIDE'):self.check()
    def test_uuid(self):
        self.body['request_id']='../../bad'
        with self.assertRaises(ValueError):self.check()
    def test_finite_shape(self):
        self.body['xyz']=[0,0]
        with self.assertRaises(ValueError):self.check()
    def test_ticket_consumed_once(self):
        t=Tickets();key=t.issue(10);self.assertTrue(t.consume(key,11));self.assertFalse(t.consume(key,11))
    def test_ticket_expiry(self):
        t=Tickets();key=t.issue(10);self.assertFalse(t.consume(key,25))
    def test_ticket_not_future(self):
        t=Tickets();key=t.issue(10);self.assertFalse(t.consume(key,9))
    def test_bounded_tickets(self):
        t=Tickets()
        for _ in range(300):t.issue(10)
        self.assertEqual(len(t.items),128)
    def test_auth_scopes_path_and_payload(self):
        s=sign('secret','/state',b'{}')
        self.assertNotEqual(s,sign('secret','/relocalize',b'{}'))
        self.assertNotEqual(s,sign('secret','/state',b'{ }'))

if __name__=='__main__':unittest.main()
