"""Recording snapshots, retention, failure isolation and evidence redaction."""
import json,tempfile,threading,unittest
from pathlib import Path
from backend.flight_log import FlightLog,navigation_sample


class FlightLogTests(unittest.TestCase):
    def test_snapshot_is_immutable_and_close_drains_queue(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'trace.jsonl';f=FlightLog(p);v={'axes':[.13,0,.2]}
            f.emit('sample',value=v);v['axes'][0]=9;f.close()
            records=[json.loads(line) for line in p.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(records[-1]['value']['axes'],[.13,0,.2])
            self.assertIn('mono',records[-1]);self.assertIn('wall_ns',records[-1]);self.assertEqual(f.written,2)
    def test_disk_rotation_has_bounded_retention_and_keeps_latest(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'trace.jsonl';f=FlightLog(p,max_bytes=500,backups=2)
            for i in range(20):f.emit('sample',seq=i,payload='x'*100)
            f.close();self.assertLessEqual(len(list(Path(d).iterdir())),3)
            records=[json.loads(line) for line in p.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(records[-1]['seq'],19);self.assertEqual(f.failed,0)
    def test_invalid_values_do_not_crash_or_corrupt_following_records(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'trace.jsonl';f=FlightLog(p)
            f.emit('invalid',value=float('nan'));f.emit('good',value=1);f.close()
            self.assertEqual(f.dropped,1);self.assertEqual(json.loads(p.read_text().splitlines()[-1])['event'],'good')
    def test_unwritable_destination_reports_failure_without_throwing(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'file';p.write_text('occupied');f=FlightLog(p/'trace.jsonl')
            f.emit('sample');f.close();self.assertIsNotNone(f.error);self.assertEqual(f.dropped,1)
    def test_full_queue_drops_records_instead_of_waiting_for_disk(self):
        with tempfile.TemporaryDirectory() as d:
            f=FlightLog(Path(d)/'trace.jsonl',capacity=1);entered=threading.Event();release=threading.Event()
            original=f.handler.handle
            def blocked(record):entered.set();release.wait(2);original(record)
            f.handler.handle=blocked;f.emit('hold');self.assertTrue(entered.wait(1))
            try:
                for i in range(30):f.emit('sample',seq=i)
                self.assertGreater(f.dropped,0)
            finally:release.set();f.close()
    def test_navigation_record_excludes_authentication_material(self):
        s=navigation_sample({'run_id':'run'},dict(pose=[0,0,0,0],private_key='secret'),{},
            dict(output=[.1,0,0],request_ticket='private-ticket',key='private-key'),{})
        encoded=json.dumps(s)
        self.assertNotIn('private',encoded);self.assertEqual(s['control']['output'],[.1,0,0])
    def test_upgrade_checkpoint_keeps_progress_without_control_actions(self):
        from mission import Mission
        from test_unified import FakeBridge,route
        b=FakeBridge();m=Mission({'indoor':route()},b)
        m.restore_paused(dict(route_id='indoor',route=m.plans.route('indoor'),target_index=3,
            reached_indices=[2],skipped_indices=[0,1],override='stairs',input_kind='velocity'))
        self.assertEqual(m.navigator.target,3);self.assertEqual(m.navigator.reached,[2])
        self.assertEqual(m.navigator.skipped,[0,1]);self.assertEqual(m.owner,'paused')
        self.assertFalse(m.execution);self.assertFalse(b.actions);self.assertEqual(b.stops,0)
    def test_invalid_upgrade_checkpoint_does_not_change_route(self):
        from mission import Mission
        from test_unified import FakeBridge,route
        m=Mission({'indoor':route()},FakeBridge())
        with self.assertRaisesRegex(ValueError,'TARGET'):
            m.restore_paused(dict(route_id='indoor',route=m.plans.route('indoor'),target_index=99))
        self.assertEqual(m.navigator.target,0)


if __name__=='__main__':unittest.main()
