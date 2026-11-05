"""No hardware: failure injection for bounded calibration motion."""
import copy,unittest
from unittest.mock import patch
from tools import e6_calibration_view as m
from tools.e6_zero import State,E6Error

START=[0.]*6;TARGET=[0.,0.,0.,0.,0.,2.]
POSE=[100.,0.,400.,0.,0.,0.]
STAGE={'start_q':START,'target_q':TARGET,'samples':[{'q':[0.,0.,0.,0.,0.,i*.25],'pose':[100.,0.,400.+i*.25,0.,0.,i*.25]} for i in range(9)]}
class Robot:
    def __init__(self,poses=None):self.commands=[];self.poses=iter(poses or [POSE]*20)
    def command(self,v):
        self.commands.append(v)
        return ','.join(map(str,next(self.poses))) if v.startswith('GetPose') else '1'
class Clock:
    def __init__(self):self.t=0.
    def __call__(self):return self.t
    def sleep(self,v):self.t+=v
class Tests(unittest.TestCase):
    def setUp(self):
        p=patch('socket.create_connection',side_effect=AssertionError('No hardware'));p.start();self.addCleanup(p.stop)
    def test_preview_and_changed_start_never_move(self):
        for mode,q in [(4,START),(5,[.2,0,0,0,0,0])]:
            r=Robot()
            with patch.object(m,'read_state',return_value=State(mode,q,0)):
                with self.assertRaises(E6Error):m.run(r,STAGE,START,{},True)
            self.assertFalse(any(v.startswith('Mov') for v in r.commands))
        r=Robot()
        with patch.object(m,'read_state',return_value=State(5,START,0)):
            self.assertEqual(m.run(r,STAGE,START,{},False),'preview')
        self.assertFalse(any(v.startswith('Mov') for v in r.commands))
    def test_path_tampering_and_excessive_travel_rejected(self):
        stage=copy.deepcopy(STAGE);stage['target_q'][-1]=3.01
        with self.assertRaises(E6Error):m.validate_stage(stage,START)
        stage=copy.deepcopy(STAGE);stage['samples'][4]['q'][-1]+=.1
        with self.assertRaises(E6Error):m.validate_stage(stage,START)
        plan={'status':'sampled_preview','origin_q':START,'origin_pose':POSE,'stages':[copy.deepcopy(STAGE)]}
        plan['stages'][0]['samples'][4]['pose'][0]+=3.1
        with self.assertRaises(E6Error):m.validate_plan(plan)
    def test_lost_feedback_stops_once_and_never_recovers(self):
        r=Robot();c=Clock()
        with patch.object(m,'read_state',side_effect=[State(5,START,0)]*2+[TimeoutError('Lost')]):
            with self.assertRaises(TimeoutError):m.run(r,STAGE,START,{},True,c,c.sleep)
        self.assertEqual(r.commands.count('Stop()'),1)
        self.assertEqual(sum(v.startswith('Mov') for v in r.commands),1)
    def test_cartesian_and_joint_departure_stop(self):
        badpose=POSE.copy();badpose[0]+=2
        badq=START.copy();badq[1]+=.2
        for q,pose in [(TARGET,badpose),(badq,POSE)]:
            r=Robot([POSE]*2+[pose]);c=Clock()
            with patch.object(m,'read_state',side_effect=[State(5,START,0)]*2+[State(7,q,0)]):
                with self.assertRaises(E6Error):m.run(r,STAGE,START,{},True,c,c.sleep)
            self.assertEqual(r.commands[-1],'Stop()')
    def test_arrival_holds_enable(self):
        r=Robot([POSE]*2+[STAGE['samples'][-1]['pose']]*5);c=Clock()
        with patch.object(m,'read_state',side_effect=[State(5,START,0)]*2+[State(5,TARGET,0)]*5):
            self.assertEqual(m.run(r,STAGE,START,{},True,c,c.sleep),'reached')
        self.assertEqual(sum(v.startswith('Mov') for v in r.commands),1)
        self.assertFalse(any(v.startswith(('Enable','Disable','Stop')) for v in r.commands))
    def test_holding_guard_failure_after_move_stops_once(self):
        r=Robot();c=Clock();calls=[0]
        def guard():
            calls[0]+=1
            if calls[0]==5:raise E6Error('Gripper drop')
        with patch.object(m,'read_state',return_value=State(5,START,0)):
            with self.assertRaises(E6Error):m.run(r,STAGE,START,{},True,c,c.sleep,guard=guard)
        self.assertEqual(sum(v.startswith('Mov') for v in r.commands),1)
        self.assertEqual(r.commands.count('Stop()'),1)
    def test_initial_holding_guard_failure_never_moves(self):
        r=Robot()
        with self.assertRaises(E6Error):m.run(r,STAGE,START,{},True,guard=lambda:(_ for _ in ()).throw(E6Error('Lost hold')))
        self.assertFalse(any(v.startswith('Mov') for v in r.commands))
if __name__=='__main__':unittest.main()
