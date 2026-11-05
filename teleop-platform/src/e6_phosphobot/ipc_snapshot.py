"""Publish changed local solver state promptly, with a bounded heartbeat.

This suppresses duplicate serialization only. LatestJsonPipe retains its
partial-write/latest-replacement behavior and the original target deadlines.
"""
from copy import deepcopy


class SnapshotPublisher:
    def __init__(self, pipe, heartbeat_ns=50_000_000):
        self.pipe = pipe
        self.heartbeat_ns = heartbeat_ns
        self.previous = None
        self.published_ns = None
        self.publications = 0

    def update(self, state, now):
        if (self.previous != state or self.published_ns is None or
                now - self.published_ns >= self.heartbeat_ns):
            self.pipe.offer(state)
            # Nested diagnostics may be changed by the producer later.
            self.previous = deepcopy(state)
            self.published_ns = now
            self.publications += 1
        # A partial write must make progress even when the state is unchanged.
        self.pipe.flush()


def solver_wait_seconds(cycle, now):
    """Bound host-input/exit polling without delaying the next 20 Hz step.

    A due request that cannot run (e.g. no feedback) still waits for incoming
    data or the normal poll timeout, instead of spinning at zero timeout.
    """
    remaining = cycle.next_request_ns - now
    return min(.005, remaining / 1e9) if cycle.pending is None and remaining > 0 else .005
