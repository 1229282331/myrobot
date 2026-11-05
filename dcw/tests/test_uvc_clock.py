import struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from uvc_clock import descriptor_clock_hz,v4l2_pts,PtsClock

def block(pts,ns=1000000000,flags=0x8c):
    return struct.pack('<QHBBIIH',ns,3,12,flags,pts,pts+5,7)

class UvcClockTests(unittest.TestCase):
    def test_descriptor_clock_and_copied_payload(self):
        interface=bytes([9,4,0,0,1,0x0e,1,0,0])
        header=struct.pack('<BBBHHIBB',13,0x24,1,0x100,13,15000000,1,1)
        self.assertEqual(descriptor_clock_hz(interface+header),15000000)
        self.assertEqual(v4l2_pts(block(123)+block(123,1000100000),1000200000),(123,1000000000))

    def test_wrap_converts_ticks_with_continuity(self):
        c=PtsClock(15000000)
        a=c.add(2**32-250000);b=c.add(250000)
        self.assertEqual(b['uvc_pts_wrap_count'],1)
        self.assertEqual(b['uvc_pts_unwrapped']-a['uvc_pts_unwrapped'],500000)
        self.assertLessEqual(abs(b['uvc_pts_ns']-a['uvc_pts_ns']-33333333),1)

    def test_reset_and_inconsistent_headers_fail(self):
        c=PtsClock(15000000);c.add(5000000)
        with self.assertRaises(ValueError):c.add(0)
        with self.assertRaises(ValueError):v4l2_pts(block(1)+block(2),1000100000)
        with self.assertRaises(ValueError):v4l2_pts(block(1)[:-1],1000100000)
        with self.assertRaises(ValueError):v4l2_pts(block(1,flags=0xcc),1000100000)

if __name__=='__main__':unittest.main()
