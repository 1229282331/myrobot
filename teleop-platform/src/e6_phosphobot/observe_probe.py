"""Passive 30004 audit using the pinned official SDK parser. No dashboard/CAN IO."""
import argparse
import base64
from dataclasses import asdict
import json
from pathlib import Path
import queue
import time

import numpy as np

from .feedback import FeedbackReader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.4')
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    feedback = FeedbackReader(args.host)
    subscription = feedback.subscribe(1024)
    joints, gaps, modes = [], [], set()
    last_ns = None
    error = None
    start = time.perf_counter()
    try:
        with feedback, (args.output / 'frames.jsonl').open('x', encoding='utf-8') as file:
            while time.perf_counter() - start < args.seconds:
                try:
                    generation, sequence, frame = subscription.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                file.write(json.dumps(dict(generation=generation, sequence=sequence,
                    received_ns=frame.received_ns, status=asdict(frame.status),
                    wire_base64=base64.b64encode(frame.prefix + frame.raw).decode())) + '\n')
                joints.append(frame.status.joint_state.q_actual)
                modes.add((frame.status.robot_mode.value, frame.status.enable_status,
                           frame.status.error_status, frame.status.collision_state,
                           frame.status.run_queued_cmd))
                if last_ns is not None:
                    gaps.append((frame.received_ns - last_ns) / 1e6)
                last_ns = frame.received_ns
    except Exception as exc:
        error = str(exc)
    summary = dict(duration_s=time.perf_counter() - start, frames=len(joints),
        modes=sorted(modes), error=error, subscriber_dropped=subscription.dropped,
        joint_peak_to_peak_deg=np.ptp(joints, axis=0).tolist() if joints else None,
        gap_ms=dict(zip(['p50','p95','p99','max'], np.percentile(gaps, [50,95,99,100]))) if gaps else None,
        final=feedback.snapshot(), reader_events=list(feedback.events),
        e6_dashboard_commands=0, can_sends=0)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in summary.items() if k != 'final'}))
    return int(bool(error or not joints or subscription.dropped))


if __name__ == '__main__':
    raise SystemExit(main())
