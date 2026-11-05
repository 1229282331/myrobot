import copy,unittest
from tools import e6_grasp_approach as m
from tools.e6_zero import E6Error

class Tests(unittest.TestCase):
    def setUp(self):
        self.plan=dict(status='sampled_noncontact_hover',origin_q=[0.]*6,origin_pose=[100,-200,400,0,0,0],origin_arm_origins_mm=[[0.,0.,0.]]*6,
                       stages=[dict(start_q=[0.]*6,target_q=[0.,0.,0.,0.,0.,2.],samples=[dict(q=[0.,0.,0.,0.,0.,i*.25],pose=[100,-200,400,0,0,i*.25],arm_origins_mm=[[0.,0.,0.]]*6) for i in range(9)])])
    def test_good_and_disconnected(self):
        m.validate_plan(self.plan);self.plan['stages'][0]['start_q'][0]=.01
        with self.assertRaises(E6Error):m.validate_plan(self.plan)
    def test_contact_path_rejected(self):
        self.plan['stages'][0]['samples'][4]['pose'][2]=269
        with self.assertRaises(E6Error):m.validate_plan(self.plan)
    def test_workspace_orientation_and_arm_sweep_rejected(self):
        for index,value in [(0,49),(1,-391),(3,40)]:
            p=copy.deepcopy(self.plan);p['stages'][0]['samples'][4]['pose'][index]=value
            with self.assertRaises(E6Error):m.validate_plan(p)
        self.plan['stages'][0]['samples'][4]['arm_origins_mm'][2]=[151,0,0]
        with self.assertRaises(E6Error):m.validate_plan(self.plan)

if __name__=='__main__':unittest.main()
