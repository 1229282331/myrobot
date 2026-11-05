"""Bounded empty-gripper comparison: fixed target versus 20 Hz small updates.

Reuses the production CAN transport and official feedback reader. No E6 writes.
Requires a separately running read-only camera service and explicit --execute.
The returned CAN state alone is never an arrival proof.
"""
import argparse
import base64
from dataclasses import asdict
import json
from pathlib import Path
import signal
import time

from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.gripper import CanDevice
from e6_phosphobot.omnipicker_can import arrived, LIMITS


def comparison(kind):
    # Fixed/ramp share endpoints. Four-count batches end one count lower (69)
    # so every update is exactly four counts; this is a separate diagnostic.
    yield 'initialize35', .35, 6.0
    if kind == 'batch-four':
        for raw in range(85, 68, -4):
            yield 'batch_close', raw / 255, 4 / (255 * .025)
        yield 'close69', 69 / 255, 6.0
        yield 'return35', .35, 6.0
        return
    if kind == 'ramp':
        target = .35
        while target > 70 / 255:
            target = max(70 / 255, target - .025 * .05)
            yield 'ramp_close', target, .05
    yield 'close70', 70 / 255, 6.0
    yield 'return35', .35, 6.0


def check_health(sample, camera, now):
    s = sample['status']
    if not sample['connected'] or s is None or not 0 <= sample['age_ms'] <= 100:
        raise RuntimeError('E6 feedback stale/disconnected')
    if not s['enable_status'] or s['error_status'] or s['collision_state'] or s['robot_mode'] != 5 or s['run_queued_cmd']:
        raise RuntimeError('E6 is not enabled, idle and fault-free')
    if camera.get('serial') != 'AY4A1520111' or camera.get('error') or camera.get('closed_ns'):
        raise RuntimeError('Camera identity or health invalid')
    if not 0 <= now - camera['rgb_received_ns'] <= 500_000_000:
        raise RuntimeError('RGB stale')


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(kind=args.kind, limits=LIMITS, request_hz=20, target_range=[69/255 if args.kind=='batch-four' else 70/255, .35],
                  e6_writes=0, stages=[], status='preflight')
    cancelled = [False]
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: cancelled.__setitem__(0, True))
    device = None
    reader = FeedbackReader('192.168.1.4')
    subscription = reader.subscribe(2048)
    with (args.output/'events.jsonl').open('x') as log, (args.output/'feedback.jsonl').open('x') as raw:
        def emit(event):
            log.write(json.dumps(dict(at_ns=time.perf_counter_ns(), **event))+'\n'); log.flush()

        def health():
            if cancelled[0]: raise RuntimeError('Diagnostic cancelled')
            while not subscription.queue.empty():
                generation, sequence, frame = subscription.queue.get_nowait()
                raw.write(json.dumps(dict(generation=generation, sequence=sequence, received_ns=frame.received_ns,
                    status=asdict(frame.status), wire_base64=base64.b64encode(frame.prefix+frame.raw).decode()))+'\n')
            if subscription.dropped: raise RuntimeError('Feedback evidence overflow')
            sample = reader.snapshot()
            check_health(sample, json.loads(args.camera_health.read_text()), time.perf_counter_ns())
            if sample['generation'] != baseline_generation[0]: raise RuntimeError('E6 feedback reconnected')
            return sample

        baseline_generation = [None]
        try:
            reader.start()
            deadline = time.monotonic()+3
            while reader.snapshot()['status'] is None and time.monotonic()<deadline: time.sleep(.01)
            baseline_generation[0] = reader.snapshot()['generation']
            before=[]
            for _ in range(100):
                before.append(health()['status']['joint_state']['q_actual']); time.sleep(.05)
            if max(max(axis)-min(axis) for axis in zip(*before)) > .01:
                raise RuntimeError('E6 did not hold a stable five-second preflight')
            report['preflight']=dict(generation=baseline_generation[0],joint_peak_to_peak_deg=[max(a)-min(a) for a in zip(*before)])
            device=CanDevice(emit)
            report['status']='running'
            for label, target, duration in comparison(args.kind):
                stage=dict(label=label,target=target,target_raw=round(target*255),feedback=[])
                report['stages'].append(stage);begin=time.monotonic();consecutive=0
                while time.monotonic()-begin < duration:
                    tick=time.monotonic();sample=health()
                    feedback=device.exchange(target)
                    stage['feedback'].append(feedback)
                    emit(dict(event='diagnostic_feedback',label=label,target=target,feedback=feedback,
                              pre_action_observation=sample))
                    consecutive=consecutive+1 if arrived(feedback,round(target*255)) else 0
                    if consecutive>=3 and 'first_arrival_s' not in stage:
                        stage['first_arrival_s']=time.monotonic()-begin
                    time.sleep(max(0, .05-(time.monotonic()-tick)))
                stage['arrived']=consecutive>=3
                print(json.dumps(dict(label=label,target_raw=stage['target_raw'],actual_raw=stage['feedback'][-1]['position_raw'],arrived=stage['arrived'])),flush=True)
                if label not in ('ramp_close','batch_close') and not stage['arrived']:
                    raise RuntimeError(f'{label}: state/position did not reach the same 1-count criterion')
            report['status']='completed'
        except Exception as exc:
            report.update(status='aborted',error=str(exc))
        finally:
            if device:
                try: device.close()
                except Exception as exc: report.update(status='aborted',cleanup_error=str(exc))
            report['last_arm_observation']=reader.snapshot()
            reader.close()
            report['reader_events']=list(reader.events)
            report['feedback_dropped']=subscription.dropped
            (args.output/'summary.json').write_text(json.dumps(report,indent=2))
    return 0 if report['status']=='completed' else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--kind',choices=['fixed','ramp','batch-four'],required=True)
    p.add_argument('--camera-health',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not args.execute: p.error('No device commands without --execute')
    raise SystemExit(run(args))
