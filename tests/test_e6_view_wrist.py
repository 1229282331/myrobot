"""Offline bounds and failure tests; real TCP connections are forbidden."""
import unittest
from unittest.mock import patch
from tools import e6_view_wrist as view
from tools.e6_zero import E6Error,State

Q=(-22.7,-105.9,136.05,-42.36,19.01,-.14)

class Robot:
    def __init__(self,reply='21'):self.commands=[];self.reply=reply
    def command(self,value):self.commands.append(value);return self.reply

class Clock:
    def __init__(self):self.t=0
    def __call__(self):return self.t
    def sleep(self,x):self.t+=x

class ViewingTests(unittest.TestCase):
    def setUp(self):
        guard=patch('socket.create_connection',side_effect=AssertionError('Real TCP forbidden'))
        guard.start();self.addCleanup(guard.stop)
    def test_fixed_origin_bound_and_single_step_bound(self):
        for start,target in [(4,5.01),(-4,-5.01),(0,2.01),(2,-1),(0,float('nan')),(1,1)]:
            with self.assertRaises(ValueError):view.plan(Q,start,target)
        start,target=view.plan(Q,3,5)
        self.assertEqual(start[:5],list(Q[:5]));self.assertEqual(target[:5],list(Q[:5]))
        self.assertAlmostEqual(target[5]-Q[5],5)
    def test_preview_never_writes(self):
        r=Robot()
        with patch.object(view,'read_state',return_value=State(5,Q,0)):
            self.assertEqual(view.run(r,list(Q),list(Q[:-1])+[Q[5]+1],{},False),'preview')
        self.assertEqual(r.commands,[])
    def test_not_enabled_or_changed_start_never_writes(self):
        for state in [State(4,Q,0),State(5,(Q[0]+.2,)+Q[1:],0)]:
            r=Robot()
            with patch.object(view,'read_state',return_value=state):
                with self.assertRaises(E6Error):view.run(r,list(Q),list(Q),{},True)
            self.assertEqual(r.commands,[])
    def test_stale_feedback_stops_once_without_retry(self):
        r=Robot();clock=Clock();target=list(Q);target[5]+=1
        with patch.object(view,'read_state',side_effect=[State(5,Q,0)]*2+[TimeoutError('stale')]):
            with self.assertRaises(TimeoutError):view.run(r,list(Q),target,{},True,clock,clock.sleep)
        self.assertEqual(r.commands,[view.move_command(target),'Stop()'])
    def test_other_axis_deviation_stops(self):
        r=Robot();clock=Clock();target=list(Q);target[5]+=1
        with patch.object(view,'read_state',side_effect=[State(5,Q,0)]*2+[State(7,(Q[0]+.2,)+Q[1:],0)]):
            with self.assertRaises(E6Error):view.run(r,list(Q),target,{},True,clock,clock.sleep)
        self.assertEqual(r.commands[-1],'Stop()')
    def test_confirmed_arrival_keeps_enable(self):
        r=Robot();clock=Clock();start,target=view.plan(Q,1,3);report={}
        with patch.object(view,'read_state',side_effect=[State(5,tuple(start),0)]*2+[State(5,tuple(target),0)]*5):
            self.assertEqual(view.run(r,start,target,report,True,clock,clock.sleep),'reached')
        self.assertEqual(r.commands,[view.move_command(target)])
        self.assertFalse(report['automatic_disable'])

if __name__=='__main__':unittest.main()
