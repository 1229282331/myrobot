"""Safety-relevant execution checks; never contacts hardware."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from tools import e6_look_down as m
from tools.e6_zero import State,E6Error

STAGE=json.loads((Path(__file__).parents[1]/'docs/evidence/look_down_20261003/down_path_readonly.json').read_text())['stages'][1]
class Robot:
    def __init__(self,poses=None):self.commands=[];self.poses=iter(poses or [STAGE['samples'][0]['pose']]*20)
    def command(self,v):
        self.commands.append(v)
        return ','.join(map(str,next(self.poses))) if v.startswith('GetPose') else '28'
class Clock:
    def __init__(self):self.t=0
    def __call__(self):return self.t
    def sleep(self,x):self.t+=x
class Tests(unittest.TestCase):
    def setUp(self):
        p=patch('socket.create_connection',side_effect=AssertionError('Hardware access forbidden'));p.start();self.addCleanup(p.stop)
    def test_preview_only_reads(self):
        r=Robot()
        with patch.object(m,'read_state',return_value=State(5,STAGE['start_q'],0)):
            self.assertEqual(m.run(r,STAGE,{},False),'preview')
        self.assertTrue(all(x.startswith('GetPose') for x in r.commands))
    def test_exact_sealed_paths_and_overlarge_stage(self):
        import hashlib,copy
        root=Path(__file__).parents[1]/'docs/evidence/look_down_20261003'
        for name,expected in [('down_path_readonly.json',m.PLAN_SHA256),('full_bottle_path.json',m.FULL_BOTTLE_PLAN_SHA256)]:
            self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(),expected)
        for key,value in [('pitch_delta_deg',-15.01),('lift_mm',50.01)]:
            stage=copy.deepcopy(STAGE);stage[key]=value
            with self.assertRaises(E6Error):m.validate_stage(stage)
    def test_changed_start_and_disabled_do_not_move(self):
        changed=list(STAGE['start_q']);changed[3]+=.2
        for mode,q in [(4,STAGE['start_q']),(5,changed)]:
            r=Robot()
            with patch.object(m,'read_state',return_value=State(mode,q,0)):
                with self.assertRaises(E6Error):m.run(r,STAGE,{},True)
            self.assertFalse(any(x.startswith('Mov') for x in r.commands))
    def test_after_motion_feedback_failure_stops_once(self):
        r=Robot();c=Clock()
        with patch.object(m,'read_state',side_effect=[State(5,STAGE['start_q'],0)]*2+[TimeoutError('lost')]):
            with self.assertRaises(TimeoutError):m.run(r,STAGE,{},True,c,c.sleep)
        self.assertEqual(r.commands.count('Stop()'),1)
        self.assertEqual(sum(x.startswith('Mov') for x in r.commands),1)
    def test_cartesian_departure_stops(self):
        p=STAGE['target_pose'].copy();p[0]+=3
        r=Robot([STAGE['samples'][0]['pose']]*2+[p]);c=Clock()
        with patch.object(m,'read_state',side_effect=[State(5,STAGE['start_q'],0)]*2+[State(7,STAGE['target_q'],0)]):
            with self.assertRaises(E6Error):m.run(r,STAGE,{},True,c,c.sleep)
        self.assertEqual(r.commands[-1],'Stop()')
    def test_joint_departure_stops(self):
        q=STAGE['start_q'].copy();q[0]+=.2
        r=Robot();c=Clock()
        with patch.object(m,'read_state',side_effect=[State(5,STAGE['start_q'],0)]*2+[State(7,q,0)]):
            with self.assertRaises(E6Error):m.run(r,STAGE,{},True,c,c.sleep)
        self.assertEqual(r.commands[-1],'Stop()')
    def test_arrival_keeps_enable_without_return(self):
        r=Robot([STAGE['samples'][0]['pose']]*2+[STAGE['target_pose']]*5);c=Clock()
        with patch.object(m,'read_state',side_effect=[State(5,STAGE['start_q'],0)]*2+[State(5,STAGE['target_q'],0)]*5):
            self.assertEqual(m.run(r,STAGE,{},True,c,c.sleep),'reached')
        self.assertEqual(sum(x.startswith('Mov') for x in r.commands),1)
        self.assertFalse(any(x.startswith(('Enable','Disable','Stop')) for x in r.commands))

if __name__=='__main__':unittest.main()
