import copy,unittest
from unittest.mock import patch
from tools import e6_observation_move as m
from tools.e6_zero import E6Error
from tests.test_e6_calibration_view import STAGE,START,POSE

class Tests(unittest.TestCase):
    def setUp(self):
        self.plan=dict(status='sampled_raised_observation',origin_q=START,origin_pose=POSE,
                       origin_arm_origins_mm=[[0.,0.,0.]]*6,stages=[copy.deepcopy(STAGE)])
        for s in self.plan['stages'][0]['samples']:s['arm_origins_mm']=[[0.,0.,0.]]*6
    def test_accepted_and_disconnected_rejected(self):
        m.validate_plan(self.plan)
        self.plan['stages'][0]['start_q'][1]=.2
        with self.assertRaises(E6Error):m.validate_plan(self.plan)
    def test_cartesian_orientation_and_model_sweeps_rejected(self):
        for key,value in [('pose',[103,0,400,0,0,0]),('pose',[100,0,441,0,0,0]),
                          ('pose',[100,0,400,26,0,0]),('arm_origins_mm',[[151,0,0]]*6)]:
            p=copy.deepcopy(self.plan);p['stages'][0]['samples'][4][key]=value
            with self.assertRaises(E6Error):m.validate_plan(p)
    def test_low_or_nonfinite_start_rejected(self):
        for z in [369,float('nan')]:
            p=copy.deepcopy(self.plan);p['origin_pose'][2]=z
            with self.assertRaises(E6Error):m.validate_plan(p)
    def test_stop_timeout_does_not_weaken_reads(self):
        d=m.ObservationDashboard('dummy',timeout=.5)
        with patch.object(m.Dashboard,'command',return_value='ok') as c:
            d.command('Stop()');self.assertEqual(c.call_args.kwargs['timeout'],8)
            d.command('GetAngle()');self.assertIsNone(c.call_args.kwargs['timeout'])

if __name__=='__main__':unittest.main()
