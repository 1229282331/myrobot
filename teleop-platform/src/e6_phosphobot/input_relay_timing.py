"""Bounded PC-local relay measurements, independent of control decisions.

Only an allowlist of correlation fields is retained; input bytes are forwarded
unchanged. A completed send is a WebSocket write, not an X5/device receipt.
"""
from collections import deque
import json
import math
import threading
import time


class InputRelayTiming:
    def __init__(self, capacity=8192, clock=time.perf_counter_ns):
        self.clock = clock
        self.rows = deque(maxlen=capacity)
        self.total = 0
        self.lock = threading.Lock()

    def add(self, connection, raw, received_ns, started_ns, finished_ns, outcome):
        row = dict(connection_id=connection, pc_received_ns=received_ns,
                   pc_send_started_ns=started_ns, pc_send_finished_ns=finished_ns,
                   outcome=outcome, characters=len(raw))
        # Diagnostics cannot make invalid or oversized input valid. Avoid
        # copying arbitrary user text, tokens, or payloads into the trace.
        if len(raw) <= 16384:
            try:
                data = json.loads(raw)
                if isinstance(data, dict):
                    seq = data.get('seq')
                    sample = data.get('client_sample_ms')
                    if type(seq) is int and 0 <= seq < 2**53:
                        row['sequence'] = seq
                    if type(sample) in (int, float) and 0 <= sample < 2**53 and math.isfinite(sample):
                        row['client_sample_ms'] = sample
            except (ValueError, RecursionError):
                pass
        with self.lock:
            self.total += 1
            self.rows.append(row)

    def snapshot(self):
        with self.lock:
            return dict(clock='pc.perf_counter_ns', capacity=self.rows.maxlen,
                        total=self.total, overwritten=self.total-len(self.rows),
                        rows=list(self.rows),
                        interpretation='Same-PC intervals only. Send completion is not remote receipt; no payloads retained.')


async def relay_inputs(ws, remote, timing, connection):
    """The existing transparent relay with timestamps, no extra await or IO."""
    async for raw in ws.iter_text():
        received_ns = timing.clock()
        started_ns = timing.clock()
        outcome = 'interrupted'
        try:
            await remote.send(raw)
            outcome = 'sent'
        finally:
            finished_ns = timing.clock()
            timing.add(connection, raw, received_ns, started_ns, finished_ns, outcome)
