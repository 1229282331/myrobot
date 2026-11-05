"""Passive shadow-only perf fault events and tcpdump ring. No audit-file tail IO."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from capture_feedback_session import bracket, read, stat


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    assert args.session.replace('-', '').isalnum()
    session = Path('/root/e6-teleop-rebuild/platform/sessions')/args.session
    assert json.loads((session/'owner.json').read_text())['stage'] == 'shadow'
    pids = {}
    for path in Path('/proc').glob('[0-9]*'):
        command = read(path/'cmdline') or ''
        if args.session not in command:
            continue
        for role in ('gateway', 'camera_service'):
            if 'e6_phosphobot.'+role+'\0' in command:
                assert role not in pids
                pids[role] = int(path.name)
    assert set(pids) == {'gateway', 'camera_service'}, pids
    assert args.output.parent.resolve() == Path('/dev/shm') and not args.output.exists()
    space = os.statvfs('/dev/shm')
    assert space.f_bavail*space.f_frsize > 256*1024**2
    args.output.mkdir()
    gateway = pids['gateway']
    identity = stat(f'/proc/{gateway}/stat')['starttime']
    commands = dict(
        tcpdump=['tcpdump', '-p', '-i', 'eth0', '-s', '0', '-B', '4096', '-U',
                 '-C', '16', '-W', '4', '-Z', 'root', '-w', str(args.output/'wire.pcap'),
                 'host 192.168.1.4 and tcp port 30004'],
        perf=['perf', 'record', '-o', str(args.output/'faults.data'), '-e', 'major-faults',
              '-c', '1', '--clockid', 'mono', '-d', '--call-graph', 'fp', '-p', str(gateway)])
    (args.output/'identity.json').write_text(json.dumps(dict(session=args.session, stage='shadow',
        pids=pids, gateway_starttime=identity, commands=commands, sample_period_s=1,
        note='perf attaches owned gateway threads and inherits its children; no system-wide profile.'), indent=2))
    before = bracket()
    start = time.monotonic()
    children = {}
    handles = {}
    halt = False
    reason = 'duration_cap'
    error = None
    samples = 0
    ready_ns = None
    def stop(*_):
        nonlocal halt
        halt = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    try:
        for name, command in commands.items():
            handles[name] = (args.output/(name+'.txt')).open('w')
            children[name] = subprocess.Popen(command, stdout=handles[name], stderr=handles[name])
        deadline = time.monotonic()+5
        while True:
            assert all(p.poll() is None for p in children.values()), 'Diagnostic child exited during startup'
            perf_fds = []
            for fd in Path('/proc', str(children['perf'].pid), 'fd').glob('*'):
                try:
                    if 'perf_event' in os.readlink(fd):
                        perf_fds.append(fd.name)
                except FileNotFoundError:
                    pass
            if perf_fds and 'listening on eth0' in (read(args.output/'tcpdump.txt') or ''):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('No perf-event descriptors or tcpdump readiness')
            time.sleep(.02)
        ready_ns = time.perf_counter_ns()
        print('capture_ready: perf major-fault events and bounded TCP ring', flush=True)
        with (args.output/'samples.jsonl').open('w') as stream:
            while time.monotonic()-start < 1200 and not halt:
                stamp = time.perf_counter_ns()
                current = stat(f'/proc/{gateway}/stat')
                if not current or current['starttime'] != identity:
                    reason = 'owned_gateway_exited'
                    break
                tasks = []
                for path in Path('/proc', str(gateway), 'task').glob('[0-9]*'):
                    value = stat(path/'stat')
                    if value:
                        tasks.append(dict(tid=int(path.name), stat=value, wchan=read(path/'wchan')))
                row = dict(started_ns=stamp, gateway=current, tasks=tasks, clock=bracket(),
                           camera=stat(f'/proc/{pids["camera_service"]}/stat'))
                row['finished_ns'] = time.perf_counter_ns()
                stream.write(json.dumps(row)+'\n')
                stream.flush()
                samples += 1
                for name, child in children.items():
                    if child.poll() is not None:
                        raise RuntimeError(name+' exited while owned gateway was still alive')
                if (args.output/'faults.data').stat().st_size > 16*1024**2:
                    reason = 'fault_evidence_size_cap'
                    break
                time.sleep(max(0, 1-(time.perf_counter_ns()-stamp)/1e9))
        if halt:
            reason = 'explicit_diagnostic_stop'
    except BaseException as exc:
        error = repr(exc)
        raise
    finally:
        for child in children.values():
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in children.values():
            child.wait(timeout=15)
        for handle in handles.values():
            handle.close()
        result = dict(before=before, after=bracket(), samples=samples,
            duration_s=time.monotonic()-start, capture_ready_ns=ready_ns, reason=reason, error=error,
            exits={name: child.returncode for name, child in children.items()})
        (args.output/'clock.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
