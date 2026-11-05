"""Same gateway/ROS/SDK parser against a retained frame over socketpair.

No E6 network, camera, CAN or device APIs are opened. Substitute command
receipts are immediate and positions remain static, so this is a contention
experiment, never a hardware motion/latency qualification.
"""
import argparse
import base64
import cProfile
import gzip
import json
import math
from pathlib import Path
import pstats
import socket
import threading
import time
from types import SimpleNamespace

import numpy as np
from e6_phosphobot.audit import Audit
from e6_phosphobot.control_state import for_operator
from e6_phosphobot.dashboard import Pending
from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.gateway import NativeExecutor, wait_control_cycle
from e6_phosphobot.moveit_bridge import MoveItBridge
from e6_phosphobot.runtime_timing import cooperative_thread_scheduling, frozen_startup_heap


class SubstituteCommands:
    uncertain = False
    _connected = True

    def __init__(self):
        self.counts = dict(ServoJ=0, Stop=0)
        self.dispatches = []

    def submit(self, name, *args, **kwargs):
        self.counts[name] += 1
        now = time.perf_counter_ns()
        self.dispatches.append(dict(name=name, at_ns=now))
        item = Pending(name, dict(command=name, sent_ns=now,
            send_completed_ns=now, acknowledged=True, accepted=True, reply_ns=now))
        item.done.set()
        return item

    def ServoJ(self, *args, **kwargs):
        return self.submit('ServoJ', *args, **kwargs)

    def Stop(self):
        return self.submit('Stop')

    def poll(self, item):
        return True

    def disconnect(self):
        self._connected = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--writers', type=int, choices=(0, 1, 2), default=2)
    parser.add_argument('--profile', action='store_true')
    parser.add_argument('--trials', type=int, default=12)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    with gzip.open(args.raw, 'rt', encoding='utf-8') as source:
        row = json.loads(next(source))
    raw = base64.b64decode(row['raw_b64'])
    left, right = socket.socketpair()
    used = False

    def fixture_socket(address, timeout):
        nonlocal used
        if address != ('retained-fixture', 30004) or used:
            raise RuntimeError('Only the one retained-frame socketpair is permitted')
        used = True
        return left

    e = NativeExecutor('retained-fixture', 'live', None,
        moveit_bundle=args.bundle, motion_profile='normal', servo_duration_s=.1)
    e.reader = FeedbackReader('retained-fixture', socket_factory=fixture_socket)
    e.dashboard = e.motion = e.robot = SubstituteCommands()
    e.camera = lambda: dict(valid=True, age_ms=0, reason='synthetic; no camera IO')
    # Do not call e.start(): live startup opens real dashboard/CAN resources.
    e.moveit = MoveItBridge(args.bundle, e.reader, motion_profile='normal')
    halt = threading.Event()
    mutex = threading.Lock()
    rows = []
    errors = []
    audit = []
    threads = []
    profiler = cProfile.Profile()
    result = dict(hardware_io=False, feedback='retained static 1440-byte SDK frame via socketpair at 125Hz',
        commands='immediate substitute receipts; no device API writes',
        writers=args.writers, profiled=args.profile, trials=args.trials,
        scope='gateway contention only; not hardware or formal-page acceptance')

    def guarded(fn):
        def run():
            try:
                fn()
            except Exception as exc:
                errors.append(repr(exc)); halt.set()
        return run

    def produce_feedback():
        while not halt.is_set():
            right.sendall(raw)
            halt.wait(.008)

    def record(event):
        for writer in audit:
            writer.put(event)

    def control():
        while not halt.is_set():
            begin = time.perf_counter_ns()
            cpu = time.thread_time_ns()
            with mutex:
                acquired = time.perf_counter_ns()
                if args.profile:profiler.enable()
                e.tick()
                if args.profile:profiler.disable()
                ended = time.perf_counter_ns()
                events = e.drain_events()
            for event in events:record(event)
            with mutex:state=e.state()
            record(dict(event='cycle',at_ns=time.perf_counter_ns(),state=state))
            rows.append(dict(event='tick',at_ns=begin,lock_wait_ms=(acquired-begin)/1e6,
                tick_ms=(ended-acquired)/1e6,total_ms=(time.perf_counter_ns()-begin)/1e6,
                thread_cpu_ms=(time.thread_time_ns()-cpu)/1e6,
                reason=state['reason'],candidate_id=(state['candidate'] or {}).get('moveit',{}).get('id'),
                take=state['model'].get('last_take')))
            wait_control_cycle(halt,e.moveit.initial_candidate_event,begin,.05,e.last_moveit_sent_ns)

    def publish():
        count=0
        while not halt.is_set():
            with mutex:state=e.state(controller=True)
            json.dumps(for_operator(state))
            if count % 4 == 0:json.dumps(state)
            count+=1
            halt.wait(.05)

    def send(op='intent', velocity=0):
        received=time.perf_counter_ns()
        with mutex:
            acquired=time.perf_counter_ns()
            data=dict(op=op,seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),
                source='keyboard',client_sample_ms=0)
            if op=='intent':data['intent']=dict(linear_m_s=[0]*3,angular_rad_s=[0]*3,
                joint_rad_s=[0]*5+[velocity],gripper_rate_s=0,deadman=bool(velocity),frame='base')
            e.receive(data)
            status=dict(ready=e.session.ready,reason=e.session.reason)
        event=dict(event='input_profile',at_ns=received,wait_ms=(acquired-received)/1e6,
            work_ms=(time.perf_counter_ns()-acquired)/1e6,op=op,velocity=velocity,**status)
        rows.append(event);record(event)

    try:
        t=threading.Thread(target=guarded(produce_feedback),name='fixture-feedback',daemon=True)
        threads.append(t);t.start();e.reader.start();e.moveit.start();e.claim('fixture')
        deadline=time.monotonic()+45
        while not e.moveit.snapshot()['ready']:
            if time.monotonic()>deadline or e.moveit.snapshot()['phase']=='failed':
                raise RuntimeError(str(e.moveit.snapshot()))
            time.sleep(.01)
        for index in range(args.writers):audit.append(Audit(args.output/f'audit-{index}',e.reader))
        for fn,name in ((control,'fixture-control'),(publish,'fixture-publisher')):
            t=threading.Thread(target=guarded(fn),name=name,daemon=True);threads.append(t);t.start()
        until=time.monotonic()+5.5
        while time.monotonic()<until:send();time.sleep(.05)
        send('prepare')
        if not e.session.ready:raise RuntimeError(e.session.reason)
        for index in range(args.trials):
            # A single new input followed by 20Hz refresh. This is a CPU
            # workload, not evidence of a keyboard, gamepad or wireless path.
            until=time.monotonic()+.5
            while time.monotonic()<until and not halt.is_set():
                send(velocity=math.radians(2)*(1 if index%2==0 else -1));time.sleep(.05)
            until=time.monotonic()+.5
            while time.monotonic()<until and not halt.is_set():send();time.sleep(.05)
            if not e.session.ready:raise RuntimeError(e.reason or e.session.reason)
        result['completed']=True
    except Exception as exc:
        result['completed']=False;errors.append(repr(exc))
    finally:
        halt.set()
        for t in threads:t.join(3)
        with mutex:
            result['bridge']=e.moveit.snapshot();result['substitute_calls']=e.dashboard.counts
            result['dispatches']=e.dashboard.dispatches
            e.close()
        right.close()
        for writer in audit:writer.close()
        result.update(errors=errors,writer_errors=[w.error for w in audit],
            threads_alive=[t.name for t in threads if t.is_alive()])
        with gzip.open(args.output/'timings.jsonl.gz','wt',encoding='utf-8') as output:
            for row in rows:output.write(json.dumps(row)+'\n')
        result['metrics']={}
        gaps=[];previous=None
        for dispatch in result['dispatches']:
            if dispatch['name']=='Stop':previous=None
            elif dispatch['name']=='ServoJ':
                if previous is not None:gaps.append((dispatch['at_ns']-previous)/1e6)
                previous=dispatch['at_ns']
        result['metrics']['dispatch_gap_ms']=dict(n=len(gaps),
            percentiles=np.percentile(gaps,[0,50,95,99,100]).tolist(),
            below_50_ms=sum(gap<50 for gap in gaps)) if gaps else None
        for kind,fields in [('tick',['lock_wait_ms','tick_ms','total_ms','thread_cpu_ms']),
                            ('input_profile',['wait_ms','work_ms'])]:
            for field in fields:
                values=[r[field] for r in rows if r['event']==kind]
                result['metrics'][kind+'.'+field]=dict(n=len(values),
                    percentiles=np.percentile(values,[50,95,99,100]).tolist()) if values else None
        if args.profile:
            profiler.dump_stats(str(args.output/'tick.prof'))
            with (args.output/'profile.txt').open('w') as stream:
                pstats.Stats(profiler,stream=stream).strip_dirs().sort_stats('cumulative').print_stats(45)
        (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k!='bridge'}),flush=True)
    return int(not result.get('completed') or bool(errors))


if __name__=='__main__':
    with cooperative_thread_scheduling(), frozen_startup_heap():
        raise SystemExit(main())
