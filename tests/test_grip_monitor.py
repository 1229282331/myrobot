import unittest
from unittest.mock import patch
from tools import grip_monitor as m

class Tests(unittest.TestCase):
    def test_stale_fault_and_cancelled_guards_reject(self):
        h=m.HoldingMonitor(None,85,{})
        with self.assertRaises(m.grip.can.CheckFailed):h.guard()
        h.count=20;h.last=10
        with patch.object(m.time,'monotonic',return_value=10.31):
            with self.assertRaises(m.grip.can.CheckFailed):h.guard()
        with patch.object(m.time,'monotonic',return_value=10.01):
            h.guard();h.error=TimeoutError('CAN lost')
            with self.assertRaises(m.grip.can.CheckFailed):h.guard()
    def test_opening_guard_fault_stall_movement_and_loss_reject(self):
        g=m.OpenGate(153);g.update(dict(fault=0,state=0,position_raw=153))
        for f in [dict(fault=1,state=0,position_raw=153),dict(fault=0,state=2,position_raw=153),
                  dict(fault=0,state=1,position_raw=153),dict(fault=0,state=0,position_raw=150)]:
            with self.assertRaises(m.grip.can.CheckFailed):g.update(f)
    def test_worker_fault_never_establishes_ready(self):
        class C:
            def exchange(self,c):raise TimeoutError('Lost device')
        h=m.HoldingMonitor(C(),85,{})
        try:
            with self.assertRaises(m.grip.can.CheckFailed):h.start()
        finally:h.stop()
        self.assertEqual(h.count,0)
    def test_nonzero_closing_target_is_preserved_for_holding(self):
        h=m.HoldingMonitor(None,85,{},target_percent=10)
        self.assertEqual(h.command[1],26)
        self.assertEqual(h.command[3],32)
        h=m.HoldingMonitor(None,85,{},target_percent=10,force_raw=64)
        self.assertEqual(h.command[3],64)
        h=m.OpeningMonitor(None,60,{},force_raw=64)
        self.assertEqual(h.command[3],64)
        with self.assertRaises(ValueError):m.HoldingMonitor(None,85,{},force_raw=65)

if __name__=='__main__':unittest.main()
