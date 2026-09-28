import unittest
from collections import deque
import numpy as np
from obstacle_evidence import clearance


class ObstacleEvidenceTests(unittest.TestCase):
    def cloud(self,points):return np.vstack([points,np.tile([3.,2.,0.],(400,1))])
    def test_recorded_right_side_cluster_does_not_block_forward(self):
        points=np.array([[.38762,-.58824,.15488],[.32884,-.61315,.15357],
            [.47972,-.54341,.17749],[.39443,-.60850,.20659],[.43931,-.60031,.24282],
            [.33719,-.59772,.21804],[.45712,-.57902,.25348],[.47893,-.57734,.25298],
            [.30858,-.61839,.24121],[.27686,-.77067,.30800]])
        r=clearance(self.cloud(points),10.,0)
        self.assertTrue(r['valid']);self.assertFalse(r['blocked'])
        self.assertEqual(r['nearby_hits'],10);self.assertEqual(r['hits'],1)
    def test_person_ahead_still_blocks(self):
        r=clearance(self.cloud([[.7,.02*i,.3] for i in range(10)]),10.,0)
        self.assertTrue(r['blocked']);self.assertEqual(r['hits'],10)
        self.assertEqual(len(r['hit_points_body']),10)
    def test_object_behind_does_not_block_forward(self):
        self.assertFalse(clearance(self.cloud([[-.7,.01*i,.3] for i in range(10)]),10.,0)['blocked'])
    def test_diagnostics_are_bounded(self):
        r=clearance(np.tile([.7,0.,.3],(1000,1)),10.,0)
        self.assertTrue(r['blocked']);self.assertEqual(r['hits'],1000)
        self.assertEqual(len(r['hit_points_body']),24)


if __name__=='__main__':unittest.main()
