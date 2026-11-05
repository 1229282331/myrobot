"""Passive shadow-only flight recorder using tcpdump's bounded native ring.

No SDK, device writes, scheduling changes or control policy. Retains a detected
slow receive and its following ten seconds; otherwise follows the owned session
until exit or the explicit 20 minute cap. Files stay in tmpfs for later review.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def read(path):
    try:
        return Path(path).read_text()
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


def bracket():
    before = time.perf_counter_ns()
    wall = time.time_ns()
    return dict(mono_before_ns=before, wall_ns=wall, mono_after_ns=time.perf_counter_ns())


def stat(path):
    text = read(path)
    if not text:
        return None
    fields = text[text.rindex(')')+2:].split()
    return dict(state=fields[0], minflt=int(fields[7]), majflt=int(fields[9]),
                utime=int(fields[11]), stime=int(fields[12]), starttime=int(fields[19]),
                processor=int(fields[36]))


def slow_receive(event):
    if event.get('event') != 'feedback_transport':
        return False
    data = event.get('data', {})
    if data.get('event') != 'receive_timing':
        return False
    return (data['publish_finished_ns']-data['recv_started_ns'] >= 100_000_000 or
            data.get('previous_publish_end_ns') is not None and
            data['recv_started_ns']-data['previous_publish_end_ns'] >= 100_000_000)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert args.session.replace('-', '').isalnum()
    session = Path('/root/e6-teleop-rebuild/platform/sessions')/args.session
    owner = json.loads((session/'owner.json').read_text())
    assert owner['stage'] == 'shadow', 'Passive diagnostic is restricted to shadow sessions'
    pids = {}
    for item in Path('/proc').glob('[0-9]*'):
        command = read(item/'cmdline') or ''
        if args.session not in command:
            continue
        for role in ('gateway', 'camera_service'):
            if 'e6_phosphobot.'+role+'\0' in command:
                assert role not in pids, 'Ambiguous process ownership'
                pids[role] = int(item.name)
    assert set(pids) == {'gateway', 'camera_service'}, pids
    assert args.output.parent.resolve() == Path('/dev/shm')
    assert not args.output.exists()
    available = os.statvfs('/dev/shm')
    assert available.f_bavail*available.f_frsize > 256*1024**2
    args.output.mkdir()
    gateway = pids['gateway']; identity = stat(f'/proc/{gateway}/stat')['starttime']
    command = ['tcpdump', '-p', '-i', 'eth0', '-s', '0', '-B', '4096', '-U',
               '-C', '16', '-W', '4', '-Z', 'root', '-w', str(args.output/'wire.pcap'),
               'host 192.168.1.4 and tcp port 30004']
    (args.output/'identity.json').write_text(json.dumps(dict(session=args.session, pids=pids,
        gateway_starttime=identity, stage='shadow', sample_period_s=.05, clk_tck=os.sysconf('SC_CLK_TCK'),
        ring_files=4, file_size_decimal_mb=16, command=command), indent=2))
    before = bracket(); start = time.monotonic(); samples = 0; reader_tid = None
    trigger = None; ending = None; reason = 'duration_cap'; partial = ''; next_audit = 0.; halt = False
    def stop(*_):
        nonlocal halt
        halt = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    audit = None; capture = None; error = None; ready_ns = None
    try:
        with (args.output/'tcpdump.txt').open('w') as log, (args.output/'samples.jsonl').open('w') as output:
            capture = subprocess.Popen(command, stdout=log, stderr=log)
            deadline=time.monotonic()+3
            while 'listening on eth0' not in (read(args.output/'tcpdump.txt') or ''):
                if capture.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('tcpdump did not confirm capture readiness')
                time.sleep(.02)
            ready_ns=time.perf_counter_ns()
            print('capture_ready', flush=True)
            while time.monotonic()-start < 1200 and not halt:
                stamp = time.perf_counter_ns()
                current = stat(f'/proc/{gateway}/stat')
                if not current or current['starttime'] != identity:
                    reason = 'owned_gateway_exited'; break
                row = dict(started_ns=stamp, gateway=current)
                if reader_tid is not None:
                    thread = Path('/proc', str(gateway), 'task', str(reader_tid))
                    row['receiver'] = dict(tid=reader_tid, stat=stat(thread/'stat'), wchan=read(thread/'wchan'))
                if samples % 20 == 0:
                    row.update(clock=bracket(), camera=stat(f'/proc/{pids["camera_service"]}/stat'),
                               meminfo=read('/proc/meminfo'), vmstat=read('/proc/vmstat'))
                if time.monotonic() >= next_audit:
                    next_audit = time.monotonic()+1
                    if audit is None and (session/'audit/events.jsonl').exists():
                        audit = (session/'audit/events.jsonl').open()
                    if audit is not None:
                        partial += audit.read(512*1024)
                        assert len(partial) <= 2*1024**2, 'Audit tail exceeded bounded diagnostic buffer'
                        lines = partial.split('\n'); partial = lines.pop()
                        for line in lines:
                            event = json.loads(line)
                            data = event.get('data', {})
                            if event.get('event') == 'feedback_transport' and data.get('reader_native_id'):
                                reader_tid = data['reader_native_id']
                            if (trigger is None and slow_receive(event) and
                                    data['recv_started_ns'] >= ready_ns):
                                trigger = dict(observed_ns=time.perf_counter_ns(), original=event)
                                ending = time.monotonic()+10
                                (args.output/'trigger.json').write_text(json.dumps(trigger, indent=2))
                                print('slow_receive_detected: retaining ten seconds after trigger', flush=True)
                row['finished_ns'] = time.perf_counter_ns()
                output.write(json.dumps(row)+'\n'); samples += 1
                if samples % 20 == 0:
                    output.flush()
                if capture.poll() is not None:
                    raise RuntimeError('tcpdump exited before diagnostic ended')
                if ending is not None and time.monotonic() >= ending:
                    reason = 'slow_receive_retained'; break
                time.sleep(max(0, .05-(time.perf_counter_ns()-stamp)/1e9))
            if halt:
                reason = 'explicit_diagnostic_stop'
    except BaseException as exc:
        error = repr(exc); raise
    finally:
        if audit is not None:
            audit.close()
        if capture is not None:
            if capture.poll() is None:
                capture.send_signal(signal.SIGINT)
            capture.wait(timeout=10)
        result = dict(before=before, after=bracket(), samples=samples, duration_s=time.monotonic()-start,
                      capture_exit=None if capture is None else capture.returncode, reason=reason, error=error,
                      trigger=trigger, feedback_thread=reader_tid, capture_ready_ns=ready_ns)
        (args.output/'clock.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
