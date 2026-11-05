"""Offline task/failure checks. Network and hardware processes are prohibited."""
import copy,json,tempfile,unittest
from datetime import datetime,timezone,timedelta
from pathlib import Path
from unittest.mock import patch,Mock
from tools import grasp_task as task
from tools.grasp_task_plan import example_bundle
from tools.grasp_task_sim import SimulatedBackend
from omnipicker.tools.grasp_can import ContactGate

class TaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.bundle=example_bundle()
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        for name in ('socket.create_connection','socket.socket','subprocess.Popen','subprocess.run'):
            p=patch(name,side_effect=AssertionError('Offline task must not access hardware'));p.start();self.addCleanup(p.stop)
    def run_task(self,*,cycles=3,backend_class=SimulatedBackend,**kwargs):
        journal=task.Journal(self.root/'run','simulation');backend=backend_class(journal,**kwargs)
        result=task.TaskRunner(self.bundle,backend,journal,cycles=cycles).run()
        events=[json.loads(line) for line in (journal.root/'events.jsonl').read_text().splitlines()]
        return result,backend,events
    def test_three_cycles_complete_and_park_with_no_hardware(self):
        r,b,e=self.run_task()
        self.assertEqual((r['status'],r['completed_cycles']),('completed',3));self.assertFalse(r['physical_success_claim'])
        self.assertEqual(b.q,self.bundle['plans']['park']['target_q']);self.assertEqual(b.stop_count,0)
        self.assertEqual(sum(x['kind']=='cycle_completed' for x in e),3)
        self.assertTrue(all(x['mode']=='simulation' for x in e))
    def test_five_cycles_complete(self):
        r,b,_=self.run_task(cycles=5);self.assertEqual(r['completed_cycles'],5);self.assertEqual(r['status'],'completed')
    def test_visual_failure_prevents_peak_release_and_park(self):
        r,b,e=self.run_task(failure_phase='verify_lift',failure_point='visual')
        self.assertEqual((r['status'],b.stop_count),('aborted',1));self.assertTrue(b.holding)
        phases=[x['phase'] for x in e if x['kind']=='phase_enter']
        self.assertNotIn('peak_lift',phases);self.assertNotIn('release',phases);self.assertNotIn('park',phases)
    def test_motion_failure_stops_before_lift(self):
        r,b,e=self.run_task(failure_phase='descend_mid',failure_point='motion_sample')
        self.assertEqual(b.stop_count,1);self.assertEqual(r['completed_cycles'],0)
        self.assertFalse(any(x['phase']=='grasp' for x in e))
    def test_gripper_contact_loss_prevents_next_motion(self):
        class Slip(SimulatedBackend):
            def guard(self):
                if self.holding and self.journal.phase=='peak_lift':self.position+=3
                super().guard()
        r,b,e=self.run_task(backend_class=Slip)
        self.assertEqual(r['status'],'aborted');self.assertEqual(b.stop_count,1)
        self.assertFalse(any(x['kind']=='robot_command' and x['phase']=='peak_lift' for x in e))
        self.assertFalse(any(x['phase']=='release' for x in e))
    def test_feedback_timeout_has_no_release(self):
        class Lost(SimulatedBackend):
            def guard(self):
                if self.journal.phase=='probe_lift':raise TimeoutError('No CAN reply within .3 seconds')
                super().guard()
        r,b,e=self.run_task(backend_class=Lost);self.assertEqual(b.stop_count,1)
        self.assertIn('TimeoutError',r['error']);self.assertTrue(b.holding);self.assertFalse(any(x['phase']=='release' for x in e))
    def test_empty_grasp_rejected_by_real_gate(self):
        class Empty(SimulatedBackend):
            def grasp(self):
                gate=ContactGate(153)
                for pos in range(149,0,-4):gate.update(self.feedback(0,pos))
                return gate.update(self.feedback(0,0))
        r,b,e=self.run_task(backend_class=Empty)
        self.assertEqual(r['status'],'aborted');self.assertEqual(b.stop_count,1)
        self.assertFalse(any(x['phase']=='probe_lift' for x in e))
    def test_cancellation_prevents_initialization(self):
        r,b,e=self.run_task(cancelled=lambda:True)
        self.assertEqual(b.stop_count,1);self.assertIsNone(b.q);self.assertEqual(r['completed_cycles'],0)
    def test_log_io_failure_while_holding_stops_once(self):
        journal=task.Journal(self.root/'run','simulation');backend=SimulatedBackend(journal);write=journal.write
        def failing(kind,**data):
            if journal.phase=='probe_lift':raise OSError('Disk full')
            return write(kind,**data)
        journal.write=failing
        r=task.TaskRunner(self.bundle,backend,journal).run()
        self.assertEqual(r['status'],'aborted');self.assertEqual(backend.stop_count,1);self.assertTrue(backend.holding)
    def test_cycle_budget_failure(self):
        clock=[0]
        class Slow(SimulatedBackend):
            def open(self):clock[0]+=601;return super().open()
        j=task.Journal(self.root/'run','simulation');b=Slow(j)
        r=task.TaskRunner(self.bundle,b,j,clock=lambda:clock[0]).run()
        self.assertEqual(b.stop_count,1);self.assertIn('budget',r['error'])
    def test_cleanup_failure_cannot_claim_success(self):
        class BadCleanup(SimulatedBackend):
            def close(self):raise OSError('Unconfirmed cleanup')
        r,b,_=self.run_task(backend_class=BadCleanup)
        self.assertEqual(r['status'],'cleanup_unconfirmed');self.assertFalse(r['physical_success_claim']);self.assertEqual(b.stop_count,1)
    def test_release_image_failure_prevents_repeat(self):
        r,b,e=self.run_task(failure_phase='verify_release',failure_point='visual')
        self.assertEqual(r['completed_cycles'],0);self.assertEqual(b.stop_count,1)
        self.assertFalse(any(x['cycle']==2 for x in e))
        self.assertFalse(any(x['phase']=='release_view_down' for x in e))
    def test_elevated_location_gate_precedes_any_descent(self):
        r,b,e=self.run_task(failure_phase='locate_target',failure_point='visual')
        self.assertEqual((r['status'],b.stop_count),('aborted',1))
        self.assertEqual(b.q,self.bundle['plans']['view_up']['stages'][-1]['target_q'])
        self.assertFalse(any(x['phase'] in ('view_down','descend_high','grasp','park') for x in e))
    def test_observation_height_and_return_are_pinned(self):
        bad=copy.deepcopy(self.bundle);bad['plans']['view_up']['stages'][-1]['samples'][-1]['pose'][2]=301
        with self.assertRaises(task.E6Error):task.validate_bundle(bad)
        bad=copy.deepcopy(self.bundle);bad['plans']['view_down']['origin_q'][0]+=1
        with self.assertRaises(task.E6Error):task.validate_bundle(bad)
    def test_bundle_sha_is_checked_before_other_access(self):
        f=self.root/'b.json';f.write_text(json.dumps(self.bundle))
        with self.assertRaisesRegex(task.E6Error,'SHA'):task.load_bundle(f,'0'*64)
    def test_offline_example_cannot_execute(self):
        f=self.root/'b.json';f.write_text(json.dumps(self.bundle))
        with self.assertRaisesRegex(task.E6Error,'cannot execute'):task.load_bundle(f,task.digest(f),execute=True)
    def test_stale_preview_and_clearance_source_changes_are_rejected(self):
        now=datetime.now(timezone.utc);b=copy.deepcopy(self.bundle)
        b['review'].update(kind='live_preview',created_utc=now.isoformat(),clearance_checked=True,clearance_checked_utc=now.isoformat(),clearance_note='Reviewed physically',source_sha256=task.source_manifest(),sampling='controller_fk')
        task.validate_execution_review(b,now)
        for field in ('created_utc','clearance_checked_utc'):
            bad=copy.deepcopy(b);bad['review'][field]=(now-timedelta(minutes=31)).isoformat()
            with self.subTest(field=field),self.assertRaises(task.E6Error):task.validate_execution_review(bad,now)
        for name,value in [('source_sha256',{}),('sampling','synthetic'),('clearance_checked',False)]:
            bad=copy.deepcopy(b);bad['review'][name]=value
            with self.subTest(name=name),self.assertRaises(task.E6Error):task.validate_execution_review(bad,now)
    def test_geometry_discontinuity_force_and_height_changes_rejected(self):
        mutations=[lambda b:b.update(force_raw=32),lambda b:b['plans']['descend_mid']['origin_q'].__setitem__(1,-80),
                   lambda b:b['plans']['probe_lift']['stages'][-1]['samples'][-1]['pose'].__setitem__(2,201),
                   lambda b:b['target_base_mm'].__setitem__(0,74),lambda b:b['plans'].pop('park')]
        for i,mutation in enumerate(mutations):
            bad=copy.deepcopy(self.bundle);mutation(bad)
            with self.subTest(i=i),self.assertRaises(task.E6Error):task.validate_bundle(bad)
    def test_invalid_cycle_count_rejected(self):
        j=task.Journal(self.root/'run','simulation');self.addCleanup(j.close)
        with self.assertRaises(task.E6Error):task.TaskRunner(self.bundle,SimulatedBackend(j),j,cycles=6)

if __name__=='__main__':unittest.main()
