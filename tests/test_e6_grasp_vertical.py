import copy,unittest
from tools.e6_grasp_vertical import validate_plan
from tools.e6_zero import E6Error

def plan():
    arms=[[0.,0.,0.] for _ in range(6)]
    return dict(status='sampled_vertical_grasp',origin_q=[0.]*6,origin_pose=[72.,-371.,240.,-180.,0.,0.],origin_arm_origins_mm=arms,
                stages=[dict(start_q=[0.]*6,target_q=[0.,0.,0.,0.,0.,1.],samples=[dict(q=[0.,0.,0.,0.,0.,i/8],pose=[72.,-371.,240.-i*5/8,-180.,0.,0.],arm_origins_mm=arms) for i in range(9)])])

class Tests(unittest.TestCase):
    def test_vertical_plan_accepted(self):validate_plan(plan())
    def test_unreviewed_table_height_axis_and_orientation_rejected(self):
        for idx,value in [(0,74.),(2,164.),(3,-178.)]:
            p=plan();p['stages'][0]['samples'][4]['pose'][idx]=value
            with self.assertRaises(E6Error):validate_plan(p)
        p=plan();p['stages'][0]['samples'][4]['pose'][2]=194.
        with self.assertRaises(E6Error):validate_plan(p)
    def test_disconnected_and_model_sweep_rejected(self):
        p=plan();p['stages'].append(copy.deepcopy(p['stages'][0]))
        with self.assertRaises(E6Error):validate_plan(p)
        p=plan();p['stages'][0]['samples'][4]['arm_origins_mm']=copy.deepcopy(p['origin_arm_origins_mm']);p['stages'][0]['samples'][4]['arm_origins_mm'][2][0]=101
        with self.assertRaises(E6Error):validate_plan(p)

if __name__=='__main__':unittest.main()
