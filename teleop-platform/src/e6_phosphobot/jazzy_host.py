"""Host-side lifecycle for the isolated Jazzy adapter. Gateway owns all IO.

Both processes run on the same Linux kernel and CLOCK_MONOTONIC. PC/device
timestamps never enter this pipe's freshness comparisons.
"""
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import threading
import time
from .latest_pipe import LatestJsonPipe
from .delivery_timing import candidate_lifetime_ns,candidate_deadline_ns


def solver_actual(sample):
    """Shared host projection for production and full-chain qualification.

    Retain original timestamps and all joint/tool fields. Raw SDK bytes,
    recording metadata and unrelated device status never enter solver IPC.
    """
    status=sample.get('status')
    actual={k:sample.get(k) for k in ('connected','age_ms','received_ns','generation')}
    actual['status']=None if not status else {k:status[k] for k in ('joint_state','tool_vector_actual')}
    return actual


class CandidateInbox:
    """One reader shared by background health polling and the control tick.

    Reads only immediately available, bounded pipe data. Decoding is outside
    the bridge lock; epoch and original deadlines remain authoritative.
    """
    def __init__(self,bridge,pipe,runtime):
        self.bridge=bridge;self.pipe=pipe;self.runtime=str(runtime)
        self.lock=threading.Lock();self.closed=False

    def receive(self):
        # The control tick may opportunistically refresh an already available
        # candidate. It must never wait behind the background pipe decoder:
        # that tick also holds the input/Stop mutex. The current candidate's
        # original age and deadline still decide whether it can be consumed.
        if not self.lock.acquire(blocking=False):return
        try:
            if self.closed:return
            row=self.pipe.receive()
            if not row:return
            bridge=self.bridge
            with bridge.lock:
                bridge.info.update(row['info'],session=self.runtime,ipc_replaced=self.pipe.replaced)
                candidate=row.get('candidate')
                if candidate is None and not row['info'].get('ready',False):
                    bridge.latest=None
                if candidate and candidate['epoch']==bridge.epoch and bridge.command is not None:
                    previous_id=bridge.latest['id'] if bridge.latest else bridge.consumed
                    new_cpp_candidate=(bridge.api=='cpp' and candidate['id']>max(bridge.consumed,previous_id))
                    if bridge.api=='cpp' and not new_cpp_candidate:
                        # Heartbeats cannot resurrect a consumed target,
                        # replace a newer one or repeatedly wake the consumer.
                        return
                    candidate['host_received_ns']=bridge.clock()
                    bridge.latest=candidate
                    now=candidate['host_received_ns']
                    fresh=(0<=now-candidate['received_ns']<candidate_lifetime_ns(candidate) and
                           now<candidate_deadline_ns(candidate))
                    first=bridge.announced_epoch!=bridge.epoch
                    expired_paced=new_cpp_candidate and (candidate.get('dispatch') or {}).get('version')==1
                    if (fresh and (first or new_cpp_candidate)) or expired_paced:
                        # Wake for an expired paced point as well: take()
                        # rejects it and cancels the outstanding parent.
                        if first and fresh:
                            bridge.announced_epoch=bridge.epoch
                            bridge.first_candidate=dict(epoch=bridge.epoch,id=candidate['id'],
                                computed_ns=candidate.get('epoch_first_computed_ns',candidate.get('computed_ns')),
                                host_received_ns=now)
                        bridge.initial_candidate_event.set()
        finally:
            self.lock.release()

    def close(self):
        with self.lock:self.closed=True


def finish_owned_group(pgid):
    def live_members():
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields=path.read_text().rsplit(')',1)[1].split()
                if fields[0]!='Z' and int(fields[2])==pgid:return True
            except (OSError,ValueError,IndexError):continue
        return False
    if not live_members():return
    try:os.killpg(pgid,signal.SIGTERM)
    except ProcessLookupError:return
    deadline=time.monotonic()+2
    while live_members() and time.monotonic()<deadline:time.sleep(.02)
    if live_members():
        try:os.killpg(pgid,signal.SIGKILL)
        except ProcessLookupError:return
        deadline=time.monotonic()+1
        while live_members() and time.monotonic()<deadline:time.sleep(.02)
        if live_members():raise RuntimeError('Owned Jazzy process group did not exit')


def isolated_runtime_argv(root, runtime, child_argv):
    """Give this owned adapter a private, RAM-backed POSIX shared-memory area.

    The rootfs is an extracted filesystem: its dev/shm is otherwise an ext4
    directory. FastDDS uses files there even with UDPv4 configured. Keep those
    operations off eMMC without altering or deleting the underlying files.
    exec preserves the dedicated process group; the kernel releases this
    private mount after the last owned descendant exits, including on failure.
    """
    root=Path(root).resolve();shm=root/'dev/shm';runtime=Path(runtime).resolve()
    if root==Path(root.anchor) or not shm.is_dir() or shm.resolve()!=shm:
        raise ValueError('Jazzy shared-memory directory must be inside its rootfs')
    if not runtime.is_dir() or not child_argv:
        raise ValueError('Jazzy runtime or child command missing')
    return ['unshare','--mount','--propagation','private','--','/bin/bash','-c',
            'set -e; mount -t tmpfs -o size=128M,mode=1777,nosuid,nodev,noexec tmpfs "$1"; '
            'test "$(stat -f -c %T "$1")" = tmpfs; '
            'findmnt --json --mountpoint "$1" > "$2"; shift 2; exec "$@"',
            'e6-jazzy-runtime',str(shm),str(runtime/'shm-mount.json'),*map(str,child_argv)]


def run(bridge):
    config=json.loads((bridge.bundle/'jazzy-runtime.json').read_text())
    root=Path(config['rootfs']).resolve()
    if not (root/'opt/ros/jazzy/setup.bash').is_file():raise ValueError('Jazzy runtime missing')
    runtime=bridge.bundle/'sessions'/str(time.time_ns());runtime.mkdir(parents=True,exist_ok=False)
    with bridge.lock:bridge.info['session']=str(runtime)
    session=config['bundle']+'/sessions/'+runtime.name
    # Paths are argv, never interpolated into a shell program. The fixed shell
    # only sources the official environment and our separately installed plugin.
    # Official Fast DDS asynchronous publishing keeps network send work off
    # the ROS callback thread. KEEP_LAST(1) and all source/trajectory deadlines
    # remain enforced; publisher return never means delivery or device ACK.
    argv=['chroot',str(root),'/usr/bin/env','ROS_LOCALHOST_ONLY=1','ROS_DOMAIN_ID=87',
          'OPENBLAS_NUM_THREADS=1','FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
          'RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS',
          '/bin/bash','-c',
          'source /opt/ros/jazzy/setup.bash; export AMENT_PREFIX_PATH="$1:$AMENT_PREFIX_PATH"; '
          'export LD_LIBRARY_PATH="$1/lib:$LD_LIBRARY_PATH"; export PYTHONPATH="$2:${PYTHONPATH:-}"; '
          'extra=(); if [ "$9" = 1 ]; then extra+=(--collision-attachments); fi; '
          'if [ "${10}" = 1 ]; then extra+=(--writer-paced); fi; '
          'exec python3 -m "$7" --bundle "$3" --session "$4" --device-period "$5" --motion-profile "$6" --tcp-profile "$8" "${extra[@]}"',
          'bash',config['plugin_prefix'],os.environ.get('E6_JAZZY_PYTHON_PATH',config['python_path']),
          config['bundle'],session,str(bridge.period),bridge.motion_profile.name,
          'e6_phosphobot.jazzy_cpp_worker' if bridge.api=='cpp' else 'e6_phosphobot.jazzy_worker',bridge.tool.name,
          '1' if bridge.collision_attachments else '0',
          '1' if bridge.api=='cpp' and bridge.writer_paced else '0']
    argv=isolated_runtime_argv(root,runtime,argv)
    with (runtime/'child.log').open('w') as log:
        bridge.process=subprocess.Popen(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,start_new_session=True)
        pipe=LatestJsonPipe(bridge.process.stdout.fileno(),bridge.process.stdin.fileno())
        inbox=CandidateInbox(bridge,pipe,runtime);bridge.refresh_candidate=inbox.receive
        previous_poll=bridge.clock()
        try:
            while not bridge.halt.is_set():
                poll_ns=bridge.clock();poll_gap_ms=(poll_ns-previous_poll)/1e6;previous_poll=poll_ns
                if bridge.process.poll() is not None:raise RuntimeError('Jazzy adapter exited')
                actual=solver_actual(bridge.reader.snapshot());command,epoch=bridge.command_snapshot()
                grip=bridge.gripper_snapshot() if getattr(bridge,'gripper_snapshot',None) else {}
                pipe.offer(dict(command=command,epoch=epoch,actual=actual,gripper=grip,at_ns=bridge.clock(),
                                dispatch=bridge.dispatch_snapshot() if bridge.api=='cpp' else None))
                pipe.flush();inbox.receive()
                with bridge.lock:bridge.info['host_poll_gap_ms']=poll_gap_ms
                if bridge.api == 'cpp':
                    # Data arriving during the poll interval is ready now;
                    # shutdown/input refresh still waits at most 5 ms.
                    select.select([pipe.read_fd], [], [], .005)
                else:
                    bridge.halt.wait(.005)
        except Exception as exc:
            with bridge.lock:
                bridge.info.update(ready=False,phase='failed',error=str(exc));bridge.latest=None
            raise
        finally:
            bridge.refresh_candidate=None;inbox.close()
            if bridge.process.poll() is None:
                bridge.process.send_signal(signal.SIGINT)
                try:bridge.process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    # The process group contains only this child and its ROS/KDL
                    # children. No global process-name based cleanup.
                    os.killpg(bridge.process.pid,signal.SIGTERM)
                    try:bridge.process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(bridge.process.pid,signal.SIGKILL);bridge.process.wait(timeout=2)
            # Also reap owned descendants when the Python child itself crashed
            # before its finally block (e.g. SIGKILL). Its original process
            # group is dedicated to this adapter, never shared with the host.
            finish_owned_group(bridge.process.pid)
            bridge.process.stdin.close();bridge.process.stdout.close()
