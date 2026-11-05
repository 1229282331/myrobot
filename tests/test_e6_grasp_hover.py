import copy,json,unittest
from pathlib import Path
from tools.e6_grasp_hover import validate
from tools.e6_zero import E6Error

class HoverTests(unittest.TestCase):
    def setUp(self):self.plan=json.loads(Path('docs/evidence/grasp_task_live_20261004/enabled_approach_plan.json').read_text())
    def test_current_sampled_path(self):validate(self.plan)
    def test_other_origin_rejected(self):
        p=copy.deepcopy(self.plan);p['origin_q'][4]+=1
        with self.assertRaises(E6Error):validate(p)
    def test_other_target_rejected(self):
        p=copy.deepcopy(self.plan);p['target_q'][1]+=1
        with self.assertRaises(E6Error):validate(p)
    def test_disconnected_path_rejected(self):
        p=copy.deepcopy(self.plan);p['stages'][4]['start_q'][3]+=.1
        with self.assertRaises(E6Error):validate(p)
    def test_sampling_gaps_rejected(self):
        p=copy.deepcopy(self.plan);p['stages'][0]['samples'].pop(4)
        with self.assertRaises(E6Error):validate(p)
    def test_lower_table_clearance_rejected(self):
        p=copy.deepcopy(self.plan);p['stages'][0]['samples'][3]['pose'][2]=275
        with self.assertRaises(E6Error):validate(p)
    def test_changed_arm_sweep_rejected(self):
        p=copy.deepcopy(self.plan);p['stages'][0]['samples'][3]['arm_origins_mm'][4][0]+=300
        with self.assertRaises(E6Error):validate(p)
    def test_large_stage_rejected(self):
        p=copy.deepcopy(self.plan);p['stages'][0]['target_q'][3]+=6
        with self.assertRaises(E6Error):validate(p)

if __name__=='__main__':unittest.main()
