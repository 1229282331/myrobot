"""Nonblocking adapter to the actual MoveIt Servo node. No device writes here.

Heavy model/ROS work stays outside the executor's tick and feedback receiver.
The gateway alone owns the actuator connection, input freshness and Stop.
"""
from collections import deque
from copy import deepcopy
from pathlib import Path
import threading
import time
from .delivery_timing import candidate_lifetime_ns,candidate_deadline_ns


class MoveItBridge:
    def __init__(self,bundle,reader,period=.05,clock=time.perf_counter_ns,motion_profile='commissioning',api='ros',tcp_profile='flange',collision_attachments=False,writer_paced=True):
        self.writer_paced=writer_paced
        from .motion_profiles import get_profile
        if api not in ('ros','cpp'):raise ValueError('Unknown MoveIt API')
        self.api=api
        self.collision_attachments=collision_attachments
        from .tool_frame import get_tool
        self.tool=get_tool(tcp_profile)
        self.motion_profile=get_profile(motion_profile)
        self.bundle=Path(bundle);self.reader=reader;self.period=period;self.clock=clock
        self.lock=threading.Lock();self.halt=threading.Event();self.thread=None
        self.epoch=0;self.command=None;self.latest=None;self.consumed=0
        self.info=dict(ready=False,phase='starting',error=None,status=None,motion_profile=self.motion_profile.description())
        self.events=deque(maxlen=256)
        self.process=None;self.kdl_process=None
        self.take_counts={};self.last_take=None
        self.refresh_candidate=None
        # ROS retains its initial notification. The fixed-step C++ producer
        # also wakes for each new candidate, so independent 20 Hz clocks do not
        # turn a slightly late result into another full-period wait. CppCycle
        # owns the calculation cadence; the gateway consumes fresh results.
        self.initial_candidate_event=threading.Event()
        self.announced_epoch=None
        self.first_candidate=None
        self.workspace=None
        self.gripper_snapshot=None
        self.geometry_lock=threading.RLock()
        self.last_arm_target=None
        self.write_receipt=None

    def dispatch_snapshot(self):
        with self.lock:return dict(version=1,receipt=deepcopy(self.write_receipt))

    def committed(self,row,sent_ns):
        with self.lock:
            self.write_receipt=dict(id=row['id'],epoch=row['epoch'],sent_ns=sent_ns,
                                    joints_rad=list(row['joints_rad']))

    def permit_gripper(self,snapshot,target):
        from .gripper_geometry import permits_motion
        with self.lock:check=deepcopy(self.info.get('gripper_check'))
        return permits_motion(check,snapshot,self.reader.snapshot(),self.clock(),round(target*255),self.last_arm_target)

    def gripper_allows_arm(self,row):
        check=row.get('cpp',{}).get('gripper_check')
        if not isinstance(check,dict) or check.get('arm_safe') is not True or self.gripper_snapshot is None:return False
        stamp=check.get('checked_ns')
        if type(stamp) is not int or not 0<=self.clock()-stamp<candidate_lifetime_ns(row):return False
        from .gripper_geometry import raw_position
        grip=self.gripper_snapshot();raw=raw_position(grip,self.clock())
        if (row.get('cpp',{}).get('margin_recovery') or {}).get('verified'):
            from .margin_recovery import writer_allows
            if not writer_allows(row,self.reader.snapshot(),grip,self.clock(),self.last_arm_target):return False
        if raw<0:return check.get('fallback') is True
        if check.get('fallback') is True:return True
        limits=check.get('arm_range_raw')
        if not isinstance(limits,list) or len(limits)!=2 or any(type(v) is not int for v in limits):return False
        lo,hi=limits
        return lo<=raw<=hi and all(grip.get(k) is None or type(grip[k]) is int and lo<=grip[k]<=hi for k in ('sent_raw','inflight_raw'))

    def start(self):
        self.thread=threading.Thread(target=self._run,name='e6-moveit-adapter',daemon=True)
        self.thread.start()

    def _run(self):
        try:
            if (self.bundle/'jazzy-runtime.json').is_file():
                from .jazzy_host import run
            else:
                if self.tool.name!='flange':raise ValueError('Grasp TCP requires Jazzy runtime')
                if self.api!='ros':raise ValueError('C++ API requires the fixed Jazzy runtime')
                if self.motion_profile.name!='commissioning':raise ValueError('Higher profiles require the qualified Jazzy route')
                from .moveit_ros import run
            run(self)
        except Exception as exc:
            import traceback
            with self.lock:
                self.info.update(ready=False,phase='failed',error=str(exc))
                self.info['traceback']=traceback.format_exc()
                self.events.append(dict(event='moveit_failed',at_ns=self.clock(),reason=str(exc),traceback=self.info['traceback']))

    def submit(self,intent,sequence):
        with self.lock:
            if self.command is None:
                self.epoch+=1;self.initial_candidate_event.clear();self.first_candidate=None
            self.command=dict(epoch=self.epoch,sequence=sequence,received_ns=self.clock(),intent=intent.model_dump())
            if self.api=='cpp' and self.workspace is not None:
                self.command['workspace']=dict(self.workspace)

    def stop(self):
        with self.lock:
            if self.command is not None:self.epoch+=1
            self.command=None;self.latest=None
            self.initial_candidate_event.clear()

    def command_snapshot(self):
        with self.lock:return deepcopy(self.command),self.epoch

    def take(self):
        # Drain bytes already available at the consumer boundary. Waiting for
        # the periodic host poll can strand a fresh candidate in the local
        # pipe until its sampling deadline. This callback never waits for data.
        started=self.clock()
        refresh=self.refresh_candidate
        if refresh is not None:refresh()
        refreshed=self.clock()
        with self.lock:
            row=self.latest;now=self.clock();locked_ns=now
            if row is None:reason='no_candidate'
            elif row['id']<=self.consumed:reason='already_consumed'
            else:
                self.consumed=row['id']
                if row['epoch']!=self.epoch or self.command is None:reason='inactive_epoch'
                elif not 0<=now-row['received_ns']<candidate_lifetime_ns(row):reason='candidate_age'
                elif now>=candidate_deadline_ns(row):reason='sample_deadline'
                else:reason='accepted'
            if reason=='accepted' and row.get('cpp',{}).get('gripper_check') and self.gripper_snapshot:
                from .gripper_geometry import raw_position
                grip=self.gripper_snapshot()
                # Feedback may arrive between acquiring the bridge lock and
                # obtaining this snapshot. Compare it with a clock read AFTER
                # the observation, then recheck the original candidate lease.
                # A slow snapshot cannot extend a deadline or authorize a
                # candidate that expired while reading the gripper state.
                now=self.clock();raw=raw_position(grip,now)
                check=row['cpp']['gripper_check']
                if not 0<=now-row['received_ns']<candidate_lifetime_ns(row):reason='candidate_age'
                elif now>=candidate_deadline_ns(row):reason='sample_deadline'
                elif raw<0:
                    if not check['fallback']:reason='gripper_feedback_expired'
                elif not check['fallback']:
                    lo,hi=check['arm_range_raw']
                    if not lo<=raw<=hi or any(type(grip.get(k)) is int and not lo<=grip[k]<=hi for k in ('sent_raw','inflight_raw')):
                        reason='gripper_path_changed'
            # Bounded diagnostics only. Never refresh a rejected timestamp or
            # make the same candidate available for a later retry.
            self.take_counts[reason]=self.take_counts.get(reason,0)+1
            self.last_take=dict(reason=reason,at_ns=now,id=row['id'] if row else None,
                sample_age_ms=(now-row['received_ns'])/1e6 if row else None,
                deadline_remaining_ms=(row['valid_until_ns']-now)/1e6 if row and 'valid_until_ns' in row else None,
                computed_ns=row.get('computed_ns') if row else None,
                host_received_ns=row.get('host_received_ns') if row else None,
                refresh_ms=(refreshed-started)/1e6,
                bridge_lock_ms=(locked_ns-refreshed)/1e6)
            if reason!='accepted':return None
            self.initial_candidate_event.clear()
            # Inbox/ROS producers publish a newly constructed candidate and
            # never mutate it afterwards. Transfer that candidate to the sole
            # consumer instead of copying its complete provenance graph under
            # the input/Stop lock. The inbox relinquishes its reference; later
            # publications and stop() cannot mutate an accepted journal row.
            self.latest=None
            return row

    def snapshot(self):
        with self.lock:return dict(self.info,epoch=self.epoch,take_counts=dict(self.take_counts),last_take=self.last_take,
                                   first_candidate=self.first_candidate)

    def close(self):
        self.stop();self.halt.set()
        if self.thread:self.thread.join(9)
        # This is after the gateway's independent Stop handling. Kill only
        # child PIDs created by this adapter, never a ROS-wide pkill.
        for proc in (self.kdl_process,self.process):
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:proc.wait(timeout=3)
                except Exception:
                    proc.kill();proc.wait(timeout=3)
        if self.thread:self.thread.join(3)
        if self.thread and self.thread.is_alive():raise RuntimeError('MoveIt adapter worker did not exit')
        with self.lock:self.info.update(ready=False,phase='closed')
