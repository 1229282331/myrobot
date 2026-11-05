"""Read-only, opt-in Gemini E RGB UVC PTS conversion on Linux V4L2.

The descriptor frequency and documented Linux UVCH blocks are used directly.
The SDK timestamp is never replaced. PTS is a device clock, not a verified
exposure instant, nor a synchronized RGB-D or robot observation.
"""
from pathlib import Path
import struct
import sys


def descriptor_clock_hz(data):
    pos, control_interface = 0, False
    clocks = set()
    while pos < len(data):
        if pos + 2 > len(data) or data[pos] < 2 or pos + data[pos] > len(data):
            raise ValueError('Malformed USB descriptors')
        item = data[pos:pos+data[pos]]
        if item[1] == 4:
            if len(item) < 9:
                raise ValueError('Short USB interface descriptor')
            control_interface = item[5:7] == b'\x0e\x01'
        elif control_interface and item[1] == 0x24 and len(item) >= 12 and item[2] == 1:
            # UVC 1.0/1.1 VC_HEADER includes dwClockFrequency at byte offset 7.
            if int.from_bytes(item[3:5], 'little') not in (0x100, 0x110):
                raise ValueError('Unsupported UVC header version for descriptor clock')
            clocks.add(int.from_bytes(item[7:11], 'little'))
        pos += len(item)
    if len(clocks) != 1 or 0 in clocks:
        raise ValueError('No unique nonzero UVC descriptor clock')
    return clocks.pop()


def gemini_rgb_clock(info, sysfs=Path('/sys/bus/usb/devices')):
    if sys.platform != 'linux' or info['vid'] != 0x2bc5 or info['pid'] != 0x065c or info['name'] != 'Gemini E':
        raise ValueError('UVC PTS option is validated only for Linux Gemini E')
    candidates = []
    for vendor in sysfs.glob('*/idVendor'):
        device = vendor.parent
        if vendor.read_text().strip() != '2bc5' or (device/'idProduct').read_text().strip() != '055c':
            continue
        if not (device/'serial').exists() or (device/'serial').read_text().strip() != info['serial_number']:
            continue
        candidates.append(device)
    if len(candidates) != 1:
        raise ValueError('Cannot uniquely match Gemini E RGB descriptor by serial number')
    device = candidates[0]
    data = (device/'descriptors').read_bytes()
    return {'clock_hz': descriptor_clock_hz(data), 'usb_device': device.name,
            'descriptor_source': str(device/'descriptors'), 'serial': info['serial_number']}


def v4l2_pts(metadata, received_ns):
    """Parse Linux UVCH blocks (host ns, SOF, copied UVC payload header)."""
    pos, pts, host_ns = 0, set(), []
    while pos < len(metadata):
        if pos+12 > len(metadata):
            raise ValueError('Truncated Linux UVC metadata block')
        ns, _sof, length, flags = struct.unpack_from('<QHBB', metadata, pos)
        if length < 2 or pos+10+length > len(metadata):
            raise ValueError('Invalid UVC metadata block length')
        if not 0 < ns <= received_ns or received_ns-ns > 3_000_000_000:
            raise ValueError('UVCH host time is not a recent CLOCK_MONOTONIC receipt')
        if flags & 0x40:
            raise ValueError('UVC packet reports a payload error')
        if flags & 4:
            if length < 6:
                raise ValueError('UVC PTS flag without four bytes')
            pts.add(struct.unpack_from('<I', metadata, pos+12)[0])
            host_ns.append(ns)
        pos += 10+length
    if len(pts) != 1:
        raise ValueError('Missing or inconsistent PTS within one RGB frame')
    return pts.pop(), min(host_ns)


class PtsClock:
    def __init__(self, hz):
        if not isinstance(hz, int) or hz <= 0:
            raise ValueError('Positive integer descriptor frequency required')
        self.hz, self.last, self.extended, self.wraps = hz, None, None, 0

    def add(self, pts):
        if not 0 <= pts < 2**32:
            raise ValueError('PTS must be uint32')
        if self.last is None:
            self.extended = pts
        else:
            delta = (pts-self.last) % 2**32
            # Continuous capture only. Reject duplicate, backwards/reset, or a
            # large discontinuity instead of misclassifying it as a wrap.
            if not 0 < delta <= self.hz*3:
                raise ValueError('UVC PTS duplicate/reset/discontinuity')
            self.wraps += pts < self.last
            self.extended += delta
        self.last = pts
        return {'uvc_pts_32': pts, 'uvc_pts_unwrapped': self.extended,
                'uvc_pts_clock_hz': self.hz, 'uvc_pts_ns': self.extended*1_000_000_000//self.hz,
                'uvc_pts_wrap_count': self.wraps}
