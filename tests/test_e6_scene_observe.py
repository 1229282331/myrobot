"""Offline path corruption/clearance boundary checks; no hardware."""
import copy,unittest
from tools.e6_scene_observe import validate_plan
from tools.e6_zero import E6Error
def plan():
    samples=[dict(q=[0,0,0,0,0,i*.25],pose=[100,0,400,0,0,i*.25],arm_origins_mm=[[0,0,0]]*6) for i in range(9)]
    return dict(status='sampled_stationary_flange_observation',origin_q=[0]*6,origin_pose=[100,0,400,0,0,0],
                origin_arm_origins_mm=[[0,0,0]]*6,stages=[dict(start_q=[0]*6,target_q=[0,0,0,0,0,2],samples=samples)])
class Tests(unittest.TestCase):
    def test_corrupted_or_unconnected_sample_rejected(self):
        for edit in [lambda p:p['stages'][0]['samples'][4]['q'].__setitem__(1,.1),
                     lambda p:p['stages'][0]['start_q'].__setitem__(0,.1)]:
            p=plan();edit(p)
            with self.assertRaises(E6Error):validate_plan(p)
    def test_sampled_clearance_and_angle_departure_rejected(self):
        for key,value in [('pose',[100,0,379,0,0,1]),('pose',[104,0,400,0,0,1]),
                          ('pose',[100,0,400,0,21,1]),('arm_origins_mm',[[0,151,0]]*6)]:
            p=plan();p['stages'][0]['samples'][4][key]=value
            with self.assertRaises(E6Error):validate_plan(p)
    def test_valid_plan_and_nonfinite_model_sample(self):
        p=plan();validate_plan(p);p['stages'][0]['samples'][4]['arm_origins_mm'][0]=[float('nan'),0,0]
        with self.assertRaises(E6Error):validate_plan(p)
if __name__=='__main__':unittest.main()
