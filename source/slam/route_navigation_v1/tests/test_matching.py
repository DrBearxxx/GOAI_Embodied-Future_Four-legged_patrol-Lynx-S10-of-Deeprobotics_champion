import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read
from s10nav.matching import Matcher,descriptor,distances

class MatchingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.m=Matcher(ROOT/'assets',read(ROOT/'config.json'))
    def test_empty_cloud_fails_closed(self):
        self.assertFalse(self.m.register(np.empty((0,3)),np.eye(4))['accepted'])
    def test_nonfinite_cloud_fails_closed(self):
        p=np.zeros((800,3));p[0,0]=np.nan
        self.assertEqual(self.m.register(p,np.eye(4))['reasons'],['nonfinite_input'])
    def test_single_plane_is_rank_deficient(self):
        x,y=np.meshgrid(np.linspace(-8,8,40),np.linspace(-8,8,40));p=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        q=self.m.quality(p,np.eye(4));self.assertLess(q['source_rank'],self.m.g['source_rank_min'])
    def test_far_outside_map_rejected(self):
        rng=np.random.default_rng(7);p=rng.normal(size=(1000,3));T=np.eye(4);T[:3,3]=1000
        self.assertFalse(self.m.register(p,T)['accepted'])
    def test_descriptor_recovers_cyclic_yaw(self):
        rng=np.random.default_rng(10);q=rng.random((20,60));g=np.roll(q,13,axis=1)[None]
        self.assertEqual(int(np.argmin(distances(q,g))),13)
    def test_map_points_match_identity(self):
        p=self.m.points[np.linalg.norm(self.m.points-np.array([.5,.1,0]),axis=1)<12]
        p=p[np.linspace(0,len(p)-1,min(len(p),5000),dtype=int)]
        r=self.m.register(p,np.eye(4));self.assertTrue(r['accepted'],r.get('reasons'))
        self.assertLess(np.linalg.norm(r['T'][:3,3]),.1)

if __name__=='__main__':unittest.main(verbosity=2)
