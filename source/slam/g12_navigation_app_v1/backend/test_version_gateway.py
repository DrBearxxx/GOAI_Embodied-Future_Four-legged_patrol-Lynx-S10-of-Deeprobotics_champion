import unittest
from unified_control import Controller,Simulator

class WaypointVersionGatewayTests(unittest.TestCase):
    def test_each_version_keeps_normalized_waypoint_input_and_full_axes(self):
        for policy in ('low','high','low_v5','high_v2'):
            sim=Simulator();c=Controller(sim);c.new_session('test')
            def command(action,seq,axes):
                c.accept(dict(action=action,axes=axes,input_kind='waypoint_axes',sample_seq=seq,sample_age_ms=0,fresh=True),10.)
            command('select:'+policy,1,[0.,0.,0.]);command('stand',2,[0.,0.,0.]);command('run',3,[0.,0.,0.])
            command('',4,[1.,-.6,.9]);c.tick(10.)
            self.assertEqual(sim.output,(1.,-.6,.9));self.assertEqual(c.status(10.)['requested'],policy)
            self.assertEqual(c.status(10.)['input_kind'],'waypoint_axes')

if __name__=='__main__':unittest.main()
