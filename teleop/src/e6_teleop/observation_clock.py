"""Age intervals from round trips, keeping independent monotonic clock epochs."""
class TransportClock:
    MAX_ANCHOR_NS = 30_000_000_000
    RELATIVE_DRIFT = .001

    def __init__(self):
        self.anchor = None
        self.candidates = []

    def update(self, sent_ns, received_ns, x5_ns):
        if received_ns < sent_ns or received_ns-sent_ns > 200_000_000:
            return False
        self.candidates = [a for a in self.candidates
                           if 0 <= received_ns-a[0] <= self.MAX_ANCHOR_NS]
        self.candidates.append((sent_ns, received_ns, x5_ns))
        if self.anchor and received_ns-self.anchor[1]<20_000_000_000 and received_ns-sent_ns>self.anchor[1]-self.anchor[0]:
            return False
        self.anchor = (sent_ns, received_ns, x5_ns)
        return True

    def bounds_ms(self, x5_sample_ns, pc_now_ns):
        intervals = []
        for sent, received, x5_anchor in self.candidates:
            elapsed = pc_now_ns-sent
            if not 0 <= elapsed <= self.MAX_ANCHOR_NS:
                continue
            x5_elapsed = x5_sample_ns-x5_anchor
            margin = max(abs(elapsed), abs(x5_elapsed))*self.RELATIVE_DRIFT
            # The reply stamp lies between PC send and receive. Intersect
            # valid intervals without assuming equal epochs or symmetric paths.
            intervals.append(((pc_now_ns-received-x5_elapsed-margin)/1e6,
                              (elapsed-x5_elapsed+margin)/1e6,
                              (received-sent)/1e6))
        if not intervals:
            return None
        lower = max(0, max(v[0] for v in intervals))
        best = min(intervals, key=lambda v: v[1])
        upper = max(0, best[1])
        if lower > upper:
            return None  # Clock discontinuity or drift outside the bound.
        return dict(age_lower_ms=lower, age_upper_ms=upper,
                    uncertainty_ms=upper-lower, round_trip_ms=best[2],
                    probes_used=len(intervals))

    def age_upper_ms(self, x5_sample_ns, pc_now_ns):
        bounds = self.bounds_ms(x5_sample_ns, pc_now_ns)
        return None if bounds is None else bounds['age_upper_ms']

    def apply(self, frame, pc_now_ns):
        bounds = self.bounds_ms(frame['observation']['gateway_monotonic_ns'], pc_now_ns)
        frame['transport'] = dict(bounds or dict(age_upper_ms=None, age_lower_ms=None,
                                                 uncertainty_ms=None, round_trip_ms=None, probes_used=0),
                                  clock_method='intersection of round-trip elapsed-interval bounds')
        age = frame['transport']['age_upper_ms']
        # X5 source age has already enforced robot=250 ms, RGB=500 ms.
        # The PC additionally bounds the transport domain at 500 ms. Do not
        # confuse RTT uncertainty with a stopped 30004 source on the X5.
        for name, limit in (('robot',500),('rgb',500),('depth',500)):
            source=frame['sources'][name]
            source['pc_age_upper_ms'] = None if age is None or source.get('age_ms') is None else age+source['age_ms']
            if source['pc_age_upper_ms'] is None or source['pc_age_upper_ms']>limit:
                source.update(valid=False,reason='传输积压或时钟探测尚未有效')
        frame['observation']['valid'] = frame['sources']['robot']['valid'] and frame['sources']['rgb']['valid'] and not frame.get('stats',{}).get('fatal')
