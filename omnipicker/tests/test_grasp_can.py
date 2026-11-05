import unittest
from omnipicker.tools import grasp_can as m

def f(pos,state=2,fault=0):return dict(position_raw=pos,state=state,fault=fault)

class Tests(unittest.TestCase):
    def test_stale_stall_empty_fault_and_drop_do_not_confirm(self):
        for feedback in [f(89),f(0,0),f(88,3),f(88,2,1)]:
            with self.assertRaises(m.can.CheckFailed):m.ContactGate(89).update(feedback)
    def test_closing_progress_then_stable_contact_required(self):
        g=m.ContactGate(100)
        for p in [98,95,91,87]:self.assertIsNone(g.update(f(p,1)))
        for _ in range(19):self.assertIsNone(g.update(f(85)))
        self.assertEqual(g.update(f(85))['position_raw'],85)
    def test_jumps_and_unstable_contact_rejected(self):
        with self.assertRaises(m.can.CheckFailed):m.ContactGate(100).update(f(70,1))
        g=m.ContactGate(100)
        for i in range(19):g.update(f(96-(i%4)))
        with self.assertRaises(m.can.CheckFailed):g.update(f(92))
    def test_hold_fault_drop_slip_and_position_loss_rejected(self):
        h=m.HoldGate(85);h.update(f(85))
        for feedback in [f(85,3),f(85,1),f(82),f(85,2,1)]:
            with self.assertRaises(m.can.CheckFailed):h.update(feedback)
    def test_cancel_and_timeout_do_not_repeat_or_next_target(self):
        class Channel:
            commands=[]
            def exchange(self,c):self.commands.append(c);raise TimeoutError('No feedback')
        c=Channel()
        with self.assertRaises(m.can.CheckFailed):m.run_stage(c,'open',35,{},stopped=lambda:True)
        self.assertEqual(len(c.commands),0)
        with self.assertRaises(TimeoutError):m.run_stage(c,'open',35,{})
        self.assertEqual(len(c.commands),1)
    def test_nonzero_target_does_not_confirm_empty_closing(self):
        class Clock:
            t=0.
            def now(self):return self.t
            def sleep(self,s):self.t+=s
        class Channel:
            def exchange(self,c):return f(c[1],0)
        clock=Clock()
        with self.assertRaisesRegex(m.can.CheckFailed,'without verified resistance'):
            m.run_stage(Channel(),'grasp',10,{},start_position=89,clock=clock.now,sleep=clock.sleep)
    def test_no_progress_watchdog_rejects_stationary_false_target_state(self):
        class Clock:
            t=0.
            def now(self):return self.t
            def sleep(self,s):self.t+=s
        class Channel:
            def exchange(self,c):return f(89,0)
        clock=Clock()
        with self.assertRaisesRegex(m.can.CheckFailed,'no measurable progress'):
            m.run_stage(Channel(),'grasp',10,{},start_position=89,clock=clock.now,sleep=clock.sleep)
        self.assertLess(clock.t,2)
    def test_release_accepts_only_brief_known_contact_then_progress(self):
        g=m.ReleaseGate(103);g.update(f(103,2),.01);g.update(f(104,1),.05);g.update(f(153,0),3.)
        for feedback,t in [(f(103,2),.3),(f(99,1),.01),(f(104,3),.1),(f(103,0),2.)]:
            with self.assertRaises(m.can.CheckFailed):m.ReleaseGate(103).update(feedback,t)
        with self.assertRaises(m.can.CheckFailed):g.update(f(103,2),.1)

if __name__=='__main__':unittest.main()
