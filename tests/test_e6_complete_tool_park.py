"""Offline rejection checks against the reviewed installed-tool path."""
import copy,json,tempfile,unittest,hashlib
from pathlib import Path
from unittest.mock import patch
from tools import e6_complete_tool_park as m
from tools.e6_zero import E6Error

PLAN=Path(__file__).resolve().parents[1]/'docs/evidence/park_complete_tool_20261003/park_plan_0004.json'

class Tests(unittest.TestCase):
    def setUp(self):
        p=patch('socket.create_connection',side_effect=AssertionError('Offline test'));p.start();self.addCleanup(p.stop)
        self.plan=json.loads(PLAN.read_text(encoding='utf-8'))
    def test_reviewed_path_accepted(self):m.validate_plan(self.plan)
    def test_disconnected_or_long_segment_rejected(self):
        for which in ['gap','long']:
            p=copy.deepcopy(self.plan)
            if which=='gap':p['stages'][1]['start_q'][2]+=.2
            else:p['stages'][0]['target_q'][2]+=1
            with self.subTest(which=which),self.assertRaises(E6Error):m.validate_plan(p)
    def test_missing_and_excessive_model_sweep_rejected(self):
        for which in ['missing','sweep']:
            p=copy.deepcopy(self.plan)
            if which=='missing':p['stages'][0]['samples'][4]['arm_origins_mm']=[]
            else:p['stages'][0]['samples'][4]['arm_origins_mm'][4][0]+=300
            with self.subTest(which=which),self.assertRaises(E6Error):m.validate_plan(p)
    def test_table_and_orientation_departure_rejected(self):
        for index,value in [(2,270),(3,0)]:
            p=copy.deepcopy(self.plan);p['stages'][0]['samples'][4]['pose'][index]=value
            with self.subTest(index=index),self.assertRaises(E6Error):m.validate_plan(p)
    def test_changed_base_or_target_rejected(self):
        for which in ['base','target']:
            p=copy.deepcopy(self.plan)
            if which=='base':p['origin_q'][0]+=.1
            else:p['target_q'][3]=-85
            with self.subTest(which=which),self.assertRaises(E6Error):m.validate_plan(p)
    def test_wrong_plan_digest_rejected_before_scene_or_hardware(self):
        with self.assertRaisesRegex(E6Error,'SHA'):m.load_inputs(PLAN,'0'*64,Path('absent'),0)
    def test_stale_and_mismatched_scene_rejected(self):
        from datetime import datetime,timezone
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);camera=root/'report.json';obs=root/'observation.json'
            scene=dict(status='completed',ended_at_utc='2020-01-01T00:00:00+00:00')
            camera.write_text(json.dumps(scene),encoding='utf-8')
            o=dict(status='static_scene',after=dict(mode=5,q=self.plan['origin_q']),source_run=str(root),camera_report_sha256=hashlib.sha256(camera.read_bytes()).hexdigest())
            obs.write_text(json.dumps(o),encoding='utf-8');digest=hashlib.sha256(PLAN.read_bytes()).hexdigest()
            with self.assertRaisesRegex(E6Error,'180 seconds'):m.load_inputs(PLAN,digest,obs,0)
            scene['ended_at_utc']=datetime.now(timezone.utc).isoformat();camera.write_text(json.dumps(scene),encoding='utf-8')
            o['camera_report_sha256']=hashlib.sha256(camera.read_bytes()).hexdigest();obs.write_text(json.dumps(o),encoding='utf-8')
            with self.assertRaisesRegex(E6Error,'stage origin'):m.load_inputs(PLAN,digest,obs,7)

if __name__=='__main__':unittest.main()
