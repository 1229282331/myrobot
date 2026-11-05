"""Recovery adapter around the existing Orbbec Camera and UVC PtsClock.

Never rewrites SDK or UVC fields. An interrupted PTS clock starts a new named
epoch rather than pretending its wrap count is continuous. Buffered frames do
not acquire a new source-receipt timestamp just because Python resumed reading.
"""
import time
import uuid


REJECTED_TIMESTAMPS = frozenset((
    'UVCH host time is not a recent CLOCK_MONOTONIC receipt',
    'UVC PTS duplicate/reset/discontinuity',
))


def source_receipt(meta):
    """Map the same-host kernel CLOCK_MONOTONIC receipt to perf_counter_ns."""
    received = meta['host_perf_counter_ns']
    kernel = meta.get('uvc_kernel_receive_ns')
    if kernel is None:
        return received, 'sdk_wait_return'
    monotonic = meta['host_monotonic_ns']
    if not 0 < kernel <= monotonic:
        raise ValueError('Invalid source kernel receipt')
    return received - (monotonic-kernel), 'uvc_kernel_receipt'


class RecoveringCapture:
    def __init__(self, camera, *, clock=time.perf_counter_ns, report=lambda event: None):
        self.camera, self.clock, self.report = camera, clock, report
        self.epoch = uuid.uuid4().hex
        self.recovering = False
        self.last_poll_ns = None
        self.stats = dict(capture_epoch=self.epoch, capture_recovering=False,
                          rejected_framesets=0, capture_recoveries=0,
                          capture_last_rejection=None, capture_poll_gaps=0)

    def gap(self, reason):
        self.stats['capture_last_rejection'] = reason
        if not self.recovering:
            self.recovering = True
            self.epoch = uuid.uuid4().hex
            self.stats.update(capture_epoch=self.epoch, capture_recovering=True)
            self.report(dict(event='capture_gap', at_ns=self.clock(), reason=reason,
                             next_epoch=self.epoch))
        # Reuse the existing validated clock implementation. No interpolation
        # across the gap, and no reuse of a previous epoch's unwrapped value.
        clock = self.camera.uvc_clock
        self.camera.uvc_clock = type(clock)(clock.hz)

    def reject(self, reason):
        self.stats['rejected_framesets'] += 1
        self.gap(reason)

    def read(self):
        now = self.clock()
        if self.last_poll_ns is not None and now-self.last_poll_ns>500_000_000:
            self.stats['capture_poll_gaps'] += 1
            self.gap('Acquisition polling interrupted for more than 500 ms')
        self.last_poll_ns = now
        try:
            frames = self.camera.read()
        except ValueError as exc:
            if str(exc) not in REJECTED_TIMESTAMPS:
                raise
            self.reject(str(exc))
            return []
        if not frames:
            return []
        prepared = []
        fresh_rgb = False
        for meta, data in frames:
            received, basis = source_receipt(meta)
            # This is the original acquisition receipt, not exposure time.
            age = self.clock()-received
            if age < 0 or age > 500_000_000:
                self.reject('Buffered acquisition receipt outside 500 ms')
                return []
            fresh_rgb |= meta['stream']=='color' and basis=='uvc_kernel_receipt'
            prepared.append((dict(meta, source_received_perf_ns=received,
                                  source_receipt_basis=basis, capture_epoch=self.epoch),data))
        if self.recovering and not fresh_rgb:
            # After a gap, depth/headers with unknown age cannot prove RGB has
            # caught up. Discard the whole frameset; don't freshen its depth.
            self.stats['rejected_framesets'] += 1
            return []
        if self.recovering:
            self.recovering = False
            self.stats['capture_recoveries'] += 1
            self.stats['capture_recovering'] = False
            self.report(dict(event='capture_recovered',at_ns=self.clock(),epoch=self.epoch,
                             first_rgb=next(meta for meta,_ in prepared if meta['stream']=='color')))
        return prepared
