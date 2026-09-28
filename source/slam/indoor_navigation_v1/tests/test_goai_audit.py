import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_goai_route import audit
from test_goai_control import data

def rows():
    return [dict(mono=i*.04,command=dict(state='ROUTE_READY',ready=True),localization=data(i*.04)[0],execute=False,sent=0,nonzero_sent=0) for i in range(801)]

class AuditTests(unittest.TestCase):
    def test_fresh_steady_readonly(self):
        r=audit(rows());self.assertTrue(r['ready_for_supervised_trial']);self.assertEqual(r['sent'],0)
    def test_no_pose_or_short_window(self):
        a=rows()
        for r in a:r['localization']=None
        self.assertFalse(audit(a)['ready_for_supervised_trial']);self.assertFalse(audit(rows()[:50])['ready_for_supervised_trial'])
    def test_hidden_log_pause_not_passed(self):
        a=rows();a=a[:300]+a[310:]
        self.assertFalse(audit(a)['ready_for_supervised_trial'])
    def test_v2_missing_samples_stay_in_denominator(self):
        from test_continuity import robust_data
        a=rows()
        for i,r in enumerate(a):r['localization']=robust_data(r['mono'])[0] if i>=10 else None
        r=audit(a);self.assertEqual(r['v2_samples'],len(a)-10)
        self.assertLess(r['v2_estimate_available_fraction'],1.)
        self.assertEqual(r['v2_conditional_available_fraction'],1.)

if __name__=='__main__':unittest.main(verbosity=2)
