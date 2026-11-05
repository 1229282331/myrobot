"""Diagnostic four-timestamp exchanges, scoped to one WebSocket connection.

Exchanges never renew input ownership, preparation or actuator commands. Their
verified intervals can conservatively bound input age; the server timestamps
must match the issued probe, and the browser contributes its own clock.
"""
from collections import OrderedDict, deque
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def input_latency_bound(client_input_ms, motion_ns, sample):
    """Offline interval; no estimate from a foreign monotonic clock alone."""
    a, d = sample['client_sent_ms'], sample['client_received_ms']
    if not finite(client_input_ms) or not d <= client_input_ms <= d + 3000:
        return None
    # Include the complete exchange as well as age since it; 1000 ppm is an
    # explicit engineering drift allowance, not certified hardware clock sync.
    error = (client_input_ms-a)*sample['relative_drift_ppm']/1e6 + .001
    elapsed = motion_ns/1e6-client_input_ms
    lower, upper = sample['offset_lower_ms'], sample['offset_upper_ms']
    return dict(lower_ms=elapsed-upper-error, upper_ms=elapsed-lower+error,
                clock_width_ms=upper-lower+2*error, drift_allowance_ms=error)


class InputAgeBounds:
    """Conservative input ages from this owner's verified clock exchanges.

    A delayed downstream state must not age a newly sampled input. Keep the
    narrowest usable proof among recent exchanges; a wide/new exchange cannot
    invalidate a still-valid earlier bound. No sample renews a motion lease.
    """
    def __init__(self):
        self.samples = deque(maxlen=8)

    def add(self, verified_sample):
        self.samples.append(dict(verified_sample))

    def bound(self, client_input_ms, now_ns):
        bounds = []
        for sample in self.samples:
            # Server age is also bounded, independently of the browser clock.
            if not 0 <= now_ns-sample['x5_sent_ns'] <= 3_000_000_000:
                continue
            bound = input_latency_bound(client_input_ms, now_ns, sample)
            if bound is not None and bound['upper_ms'] >= 0:
                bounds.append(bound)
        return min(bounds, key=lambda item:item['upper_ms']) if bounds else None


class ClockProbes:
    def __init__(self):
        self.pending = OrderedDict()

    def issue(self, client_ms, received_ns, sent_ns):
        if not finite(client_ms) or client_ms < 0:
            return None
        probe = dict(kind='clock_probe', client_sent_ms=client_ms,
                     x5_received_ns=received_ns, x5_sent_ns=sent_ns)
        self.pending[client_ms] = probe
        while len(self.pending) > 8:
            self.pending.popitem(last=False)
        return probe

    def complete(self, data, now_ns):
        a = data.get('client_sent_ms')
        if not finite(a):
            return None
        probe = self.pending.pop(a, None)
        if probe is None or not 0 <= now_ns - probe['x5_sent_ns'] <= 3_000_000_000:
            return None
        b, c = probe['x5_received_ns'], probe['x5_sent_ns']
        d = data.get('client_received_ms')
        if (data.get('x5_received_ns') != b or data.get('x5_sent_ns') != c
                or not finite(d) or d < a or d - a > 3000 or c < b):
            return None
        lower, upper = c / 1e6 - d, b / 1e6 - a
        if lower > upper:
            return None
        return dict(client_sent_ms=a, client_received_ms=d, x5_received_ns=b,
                    x5_sent_ns=c, offset_lower_ms=lower, offset_upper_ms=upper,
                    network_rtt_ms=upper-lower, relative_drift_ppm=1000)
