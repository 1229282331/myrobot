"""Offline safety checks for the single-axis commissioning steps."""
import unittest
from unittest.mock import patch
from tools import e6_step as step
from tools.e6_zero import E6Error, State

Q=(-22.7,-105.9,136.05,-43.22,19.88,.71)

class Robot:
    def __init__(self,reply='12'):
        self.commands=[]
        self.reply=reply
    def command(self,value,**kw):
        self.commands.append(value)
        return self.reply if value.startswith('MovJ(') else ''

class Clock:
    def __init__(self):self.t=0
    def __call__(self):return self.t
    def sleep(self,x):self.t+=x

class StepTests(unittest.TestCase):
    def setUp(self):
        guard=patch('socket.create_connection',side_effect=AssertionError('Real TCP forbidden'))
        guard.start();self.addCleanup(guard.stop)
        self.clock=Clock()
    def run_step(self,robot,action,samples,execute=True):
        report={}
        states=[State(mode,tuple(q),0) for mode,q in samples]
        with patch.object(step,'read_state',side_effect=states):
            result=step.run_step(robot,Q,action,report,execute,self.clock,self.clock.sleep)
        return result,report
    def test_preview_never_enables(self):
        r=Robot();result,_=self.run_step(r,'enable',[(4,Q)],False)
        self.assertEqual(result,'preview');self.assertEqual(r.commands,[])
    def test_motion_requires_existing_enable(self):
        r=Robot()
        with self.assertRaises(E6Error):self.run_step(r,'out',[(4,Q)])
        self.assertEqual(r.commands,[])
    def test_changed_baseline_never_writes(self):
        r=Robot();q=(Q[0]+.2,)+Q[1:]
        with self.assertRaises(E6Error):self.run_step(r,'enable',[(4,q)])
        self.assertEqual(r.commands,[])
    def test_hold_never_disables(self):
        r=Robot();result,report=self.run_step(r,'enable',[(4,Q)]+[(5,Q)]*60)
        self.assertEqual(result,'enabled_and_held')
        self.assertGreaterEqual(report['hold_seconds'],5)
        self.assertEqual(r.commands,['EnableRobot()'])
    def test_return_preserves_other_joint_coordinates(self):
        _,target=step.expected_joints(Q,'out')
        r=Robot();result,_=self.run_step(r,'return',[(5,target)]*2+[(7,target)]+[(5,Q)]*5)
        self.assertEqual(result,'reached')
        self.assertEqual(r.commands,[step.move_command(Q)])
        self.assertIn('a=1,v=1,cp=0',r.commands[0])
    def test_bad_queue_reply_stops_without_retry(self):
        r=Robot('not a queue ID')
        with self.assertRaises(E6Error):self.run_step(r,'out',[(5,Q)]*2)
        self.assertEqual(r.commands[-1],'Stop()');self.assertEqual(len(r.commands),2)
    def test_uncommanded_joint_motion_stops(self):
        r=Robot();q=(Q[0]+.2,)+Q[1:]
        with self.assertRaises(E6Error):self.run_step(r,'out',[(5,Q)]*2+[(7,q)])
        self.assertEqual(r.commands[-1],'Stop()')
    def test_connection_failure_stops_once(self):
        r=Robot()
        with patch.object(step,'read_state',side_effect=[State(5,Q,0),State(5,Q,0),TimeoutError('feedback stale')]):
            with self.assertRaises(TimeoutError):step.run_step(r,Q,'out',{},True,self.clock,self.clock.sleep)
        self.assertEqual(r.commands[-1],'Stop()');self.assertEqual(len(r.commands),2)
    def test_each_arm_axis_is_restricted_to_a_quarter_degree(self):
        for axis in range(1,6):
            start,target=step.expected_joints(Q,'out',axis,.25)
            self.assertEqual(start,Q)
            self.assertEqual([round(a-b,8) for a,b in zip(target,Q)],[.25 if i==axis-1 else 0 for i in range(6)])
            with self.assertRaises(ValueError):step.expected_joints(Q,'out',axis,1)
        for axis,delta in [(0,.25),(7,.25),(2,-.25),(6,180)]:
            with self.assertRaises(ValueError):step.expected_joints(Q,'out',axis,delta)
    def test_arm_axis_return_uses_reviewed_start_and_preserves_other_axes(self):
        start,target=step.expected_joints(Q,'return',2,.25)
        r=Robot();states=[State(5,start,0)]*2+[State(7,start,0)]+[State(5,target,0)]*5
        with patch.object(step,'read_state',side_effect=states):
            result=step.run_step(r,Q,'return',{},True,self.clock,self.clock.sleep,axis=2,delta=.25)
        self.assertEqual(result,'reached')
        self.assertEqual(r.commands,[step.move_command(Q)])

if __name__=='__main__':unittest.main()
