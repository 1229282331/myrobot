"""X5 loopback gateway for the phosphobot adapter. Default stage is shadow.

Native ServoP/ServoJ use official SDK builders. This module owns intent policy,
not a kinematics solver. Camera/encoder/recording processes are independent.
"""
import argparse
import asyncio
from dataclasses import asdict
from collections import deque
from contextlib import asynccontextmanager
import json
import math
from pathlib import Path
import signal
import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed
from dobot_sdk.api.motion import Motion
from dobot_sdk.api.robot_control import RobotControl

from .dashboard import Dashboard, SubmissionPort, MotionDeadlineExpired
from .feedback import FeedbackReader
from .intent import IntentSession, Limits, Message
from .control_state import for_operator
from .state_observation import StateObservation, render_observation, observe_executor, capture_audit_observation
from .clock_probe import ClockProbes
from .workspace import TestWorkspace


@asynccontextmanager
async def controller_admission(slot, timeout_s=.5):
    """Serialize opening with the previous handler's completed cleanup.

    Tab/source replacement can arrive before the old SSH websocket has closed.
    This only waits for ownership to become vacant, never evicts a controller,
    renews an intent, or retries a command. A second active controller is still
    refused after the bounded opening interval. Observers do not use this slot.
    """
    try:
        await asyncio.wait_for(slot.acquire(), timeout_s)
    except asyncio.TimeoutError:
        yield False
        return
    try:
        yield True
    finally:
        slot.release()


def pose_of(sample):
    p=sample['status']['tool_vector_actual']
    return (np.array([p[k] for k in ('x','y','z')])/1000,
            Rotation.from_euler('xyz',[p[k] for k in ('rx','ry','rz')],degrees=True))


def evaluate_workspace(base, actual, candidate, limits):
    """Site test envelope, independent of MoveIt's physical joint/collision limits.

    Stop can settle just outside the requested envelope. Keep the same origin:
    a candidate there must reduce an existing violation without increasing any
    other one. This admits retreat, never an outward expansion or outside hold.
    Values are normalized by the immutable, explicitly configured session range.
    """
    if limits.mode == 'model':
        return dict(allowed=True, retreat=False, mode='model', exceeded=[],
                    policy='moveit_kinematics_collision_and_absolute_joints')
    bp, br, bq = base
    def distances(pose):
        p, r, q = pose
        return [math.dist(p,bp)/limits.position_m,
                (r*br.inv()).magnitude()/math.radians(limits.rotation_deg) if limits.rotation_deg is not None else 0.,
                *([abs(x-y)/math.radians(limits.joint_deg) for x,y in zip(q,bq)]
                  if limits.joint_deg is not None else [0.]*6)]
    before, after = distances(actual), distances(candidate)
    old_excess, new_excess = [max(x-1,0) for x in before], [max(x-1,0) for x in after]
    retreat = (any(x>0 for x in old_excess) and all(n<=o for n,o in zip(new_excess,old_excess))
               and any(n<o for n,o in zip(new_excess,old_excess)))
    return dict(allowed=all(x<=1 for x in after) or retreat, retreat=retreat,
                actual_normalized=before, candidate_normalized=after,
                exceeded=[name for name, value in zip(
                    ['position', 'rotation', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6'], after) if value>1])


def wait_control_cycle(halted,wake,begin_ns,period_s,last_sent_ns,clock=time.perf_counter_ns,*,producer_paced=False,scheduled_ns=None):
    """Wake on availability; the C++ producer owns its motion cadence.

    Legacy producers retain last-send padding. For those paths, a fast
    release/repress notification cannot pull a device send earlier than
    one period after the previous send. Ordinary ticks retain their existing
    measured jitter rather than adding tick work to every 50 ms interval.
    """
    remaining=max(0,period_s-(clock()-begin_ns)/1e9)
    if scheduled_ns is not None:remaining=min(remaining,max(0,(scheduled_ns-clock())/1e9))
    if wake is None:
        halted.wait(remaining)
    elif wake.wait(remaining):
        wake.clear()
        if last_sent_ns is not None and not producer_paced:
            halted.wait(max(0,(last_sent_ns+round(period_s*1e9)-clock())/1e9))


class NativeExecutor:
    def __init__(self,host,stage,camera_health,*,clock=time.perf_counter_ns,acceptance_inputs=False,
                 servo_period_s=.05,servo_duration_s=None,servo_aheadtime=50,servo_gain=500,release_mode='immediate',moveit_bundle=None,
                 motion_profile='commissioning',camera_session=None,workspace_config=None,native_smoothing='none',moveit_api='ros',tcp_profile='flange',payload_config=None):
        from .tool_frame import get_tool
        from .payload import Payload
        self.payload=Payload.from_config(payload_config)
        self.tool=get_tool(tcp_profile)
        if tcp_profile!='flange' and not moveit_bundle:
            raise ValueError('Grasp TCP requires MoveIt; controller Tool0 must remain unchanged')
        if moveit_api not in ('ros','cpp') or (moveit_api=='cpp' and not moveit_bundle):
            raise ValueError('C++ API requires the MoveIt route')
        self.moveit_api=moveit_api
        self.workspace=TestWorkspace.from_config(workspace_config)
        self.assembly_id=None
        if self.workspace.mode=='model':
            if moveit_api!='cpp' or tcp_profile!='omnipicker-r4':
                raise ValueError('Model workspace requires C++ MoveIt and the reviewed R4 assembly')
            from .assembly_model import verified_assets
            self.assembly_id=verified_assets()['model_id']
        self.joint_range=None
        if self.workspace.uses_model_range:
            if not moveit_bundle:
                raise ValueError('Model workspace range requires MoveIt and verified absolute joint limits')
            from .joint_range import JointRange
            self.joint_range=JointRange(moveit_bundle)
        from .motion_profiles import get_profile
        self.motion_profile=get_profile(motion_profile)
        from .joint_tracking import JointTrackingGuard
        self.joint_tracking=JointTrackingGuard(self.motion_profile.description()['solver_joint_deg_s'],
            self.motion_profile.description()['solver_joint_acceleration_deg_s2'])
        self.last_solver_rejection_id=0
        if native_smoothing not in ('none','ruckig'):
            raise ValueError('Unknown native smoothing')
        if native_smoothing!='none' and (moveit_bundle or release_mode!='immediate'):
            raise ValueError('Native input smoothing requires native route and immediate Stop')
        if motion_profile!='commissioning' and not moveit_bundle and native_smoothing!='ruckig':
            raise ValueError('Higher native motion profiles require Ruckig input smoothing')
        self.native_smoothing=native_smoothing
        if release_mode not in ('immediate','ruckig'):
            raise ValueError('Unknown release mode')
        self.release_mode=release_mode
        if moveit_bundle and release_mode!='immediate':raise ValueError('MoveIt route uses independent immediate Stop')
        self.moveit_bundle=moveit_bundle
        self.moveit=None
        self.last_moveit_candidate_ns=None
        self.last_moveit_sent_ns=None
        self.queued_moveit=None
        self.command_port=None
        self.native_wake=threading.Event()
        self.native_started=False
        self.last_native_sent_ns=None
        self.trajectory=None
        self.braking=False
        self.brake_finished=False
        if servo_period_s not in (.03,.05):
            raise ValueError('Servo period must be an explicit qualified experiment: 0.03 or 0.05 s')
        self.servo_period_s=servo_period_s
        # The SDK's t is a point's run time, not the sender's cadence. Keep
        # the existing value by default; qualify t=0.1 without changing the
        # integrator, MoveIt sampling, input freshness or Stop scheduling.
        duration=servo_period_s if servo_duration_s is None else servo_duration_s
        if type(duration) not in (int,float) or duration not in (.03,.05,.1):
            raise ValueError('Servo duration qualification must be 0.03, 0.05 or 0.1 s')
        self.servo_duration_s=duration
        if servo_aheadtime not in (20,50):
            raise ValueError('Servo aheadtime must be 20 or 50 for this qualification')
        self.servo_aheadtime=servo_aheadtime
        if servo_gain not in (500,750):
            raise ValueError('Servo gain must be an explicit qualification: 500 or 750')
        self.servo_gain=servo_gain
        self.clock=clock
        self.acceptance_inputs=acceptance_inputs
        self.stage=stage
        self.camera_health=camera_health
        self.camera_io=None
        from .camera_restart import CameraRestart
        self.camera_restart=CameraRestart(camera_health,camera_session,clock=clock) if camera_health and camera_session else None
        self.reader=FeedbackReader(host)
        self.host=host
        self.dashboard=None
        self.motion=None
        self.robot=None
        self.session=None
        self.owner=None
        self.pending_input_cancellation=None
        self.input_cancellation_lock=threading.Lock()
        self.limits=self.motion_profile.input_limits()
        self.gripper=None
        self.dynamic_gripper=False
        self.pending=None
        self.stop_pending=None
        self.generation=None
        self.base=None
        self.flange_base=None
        self.target=None
        self.moving=False
        self.reason=None
        self.last_snapshot=None
        self.candidate=None
        self.accepted=None
        self.last_receipt=None
        self.recording_request=None
        self.stop_acknowledged=None
        self.tracking_stop_receipt_ns=None
        self.stationary=None
        self.idle_samples=deque()
        self.idle_signature=None
        self.enable_pending=None
        self.enable_phase='idle'
        self.enable_reason=None
        self.enable_receipt=None
        self.enable_started_ns=None
        self.enable_generation=None
        self.stop_failed=False
        self.blocked_signature=None
        self.last_tick_ns=None
        self.events=deque(maxlen=4096)
        self.episodes=None

    def start(self):
        self.reader.start()
        try:
            if self.camera_health is not None:
                from .latest_json_files import LatestJsonFiles
                paths=[self.camera_health]
                if self.camera_restart:paths.append(self.camera_restart.status_path)
                self.camera_io=LatestJsonFiles(paths)
                self.camera_io.start()
                if self.camera_restart:self.camera_restart.status_reader=self.camera_io.read
            if self.moveit_bundle:
                from .moveit_bridge import MoveItBridge
                self.moveit=MoveItBridge(self.moveit_bundle,self.reader,self.servo_period_s,clock=self.clock,api=self.moveit_api,tcp_profile=self.tool.name,
                                         motion_profile=self.motion_profile.name,collision_attachments=self.workspace.mode=='model',writer_paced=self.stage=='live')
                self.moveit.start()
            if self.stage=='live':
                self._connect_dashboard()
                from .gripper import Gripper
                from .gripper_geometry import geometry
                dynamic=self.workspace.mode=='model' and geometry() is not None
                self.dynamic_gripper=dynamic
                if dynamic and not self.moveit:raise ValueError('Dynamic gripper geometry requires MoveIt')
                self.gripper=Gripper(rate_s=self.limits.gripper_rate_s,motion_guard=self.moveit.permit_gripper if dynamic else None,
                                     geometry_lock=self.moveit.geometry_lock if dynamic else None)
                if self.moveit:self.moveit.gripper_snapshot=self.gripper.snapshot
                self.gripper.start()
        except Exception:
            try:
                if self.moveit:self.moveit.close()
            finally:
                try:self.reader.close()
                finally:
                    if self.camera_io:self.camera_io.close()
            raise

    def _connect_dashboard(self):
        if self.dashboard is not None:
            self.dashboard.disconnect()
        self.dashboard=Dashboard(self.host)
        # A candidate can arrive before the preceding command's reply. Wake
        # again on that reply instead of consuming availability then sleeping
        # a full cycle with an already-computed candidate waiting behind it.
        if self.moveit is not None and self.moveit_api=='cpp':
            self.dashboard.completion_wake=self.moveit.initial_candidate_event
        self.dashboard.connect()
        port=SubmissionPort(self.dashboard)
        self.command_port=port
        self.motion=Motion(port)
        self.robot=RobotControl(port)

    def camera(self):
        if self.camera_health is None:
            return dict(valid=False,reason='相机采集尚未接入')
        try:
            data=self.camera_io.read(self.camera_health) if self.camera_io else json.loads(self.camera_health.read_text())
            age=(self.clock()-data['rgb_received_ns'])/1e6
            return dict(valid=data['serial']=='AY4A1520111' and 0<=age<=500 and not data.get('error') and not data.get('closed_ns') and not data.get('capture_recovering'),
                        age_ms=age,frame_id=data.get('frame_id'),reason=data.get('error'),
                        latency_marker=data.get('latency_marker',False),
                        preview_codec=data.get('preview_codec','h264'),
                        capture_epoch=data.get('capture_epoch'),capture_recovering=data.get('capture_recovering',False))
        except Exception as exc:
            return dict(valid=False,reason=str(exc))

    def healthy(self,sample,*,require_enabled=True):
        s=sample['status']
        if s is None or not sample['connected'] or sample['age_ms'] is None or not 0<=sample['age_ms']<=100:
            return '机械臂反馈过期'
        if self.stage=='live' and require_enabled and not s['enable_status']:return '机械臂未使能'
        if s['error_status'] or s['collision_state']:return '控制器报警或碰撞'
        allowed_modes=(4,5,7) if self.stage=='shadow' or not require_enabled else (5,7)
        if s['robot_mode'] not in allowed_modes:return '控制器模式不可运动'
        if s['user_coordinate']!=0 or s['tool_coordinate']!=0 or any(abs(v)>1e-6 for v in s['user_value']+s['tool_value']):
            return '当前接入要求零偏移User0/Tool0'
        if self.stage=='live' and require_enabled:
            reason=self.payload.mismatch(s)
            if reason:return reason
        if self.camera_restart and self.camera_restart.status()['phase'] in ('requested','starting'):
            return '相机正在重新连接'
        if self.stage=='live' and not self.camera()['valid']:return 'RGB画面未更新'
        return None

    def camera_restart_unavailable(self):
        if self.camera_restart is None:return '当前启动方式未配置相机恢复'
        if self.camera_restart.status()['phase'] in ('requested','starting'):return '相机正在重新连接'
        if self.episodes and self.episodes.state()['phase'] in ('starting','capturing','sealing','transfer_pending'):
            return '请先结束录制并完成回传'
        sample=self.reader.snapshot()
        if not sample.get('connected') or sample.get('status') is None or sample.get('age_ms') is None or not 0<=sample['age_ms']<=100:
            return '等待新鲜机械臂反馈'
        if (not self.stationary or self.moving or self.pending is not None or self.stop_pending is not None
                or self.braking or self.stop_failed or self.enable_pending is not None):
            return '请释放操作并等待机械臂静止'
        return None

    def claim(self,owner):
        with self.input_cancellation_lock:
            if self.owner is not None:return False
            self.owner=owner
            self.pending_input_cancellation=None
            self.session=IntentSession(self.limits,clock=self.clock)
        self.reason=None
        return True

    def recording_health(self):
        reason=self.healthy(self.reader.snapshot())
        if reason:return reason
        if not self.camera()['valid']:return '录制的RGB源已过期'
        return None

    def check_recording(self):
        # Called under the control lock. Sealing retains the settled window;
        # once sealed, media transfer cannot revoke newly prepared control.
        if not self.episodes:return
        phase=self.episodes.state()['phase']
        if phase=='capturing':
            reason=self.recording_health()
            if reason:self.episodes.abort(reason)
        if phase=='sealing' and self.session and self.session.ready:
            self.suspend('recording_finished')

    def notify_received(self,owner,data):
        """Cancellation only, from the owning socket before the execution lock.

        This cannot accept/renew an input or prepare a session. The immutable
        marker prevents a locked computation from committing an old target
        while its release waits to enter receive(). Already submitted writes
        still require their original receipts and the independent Stop path.
        """
        try:message=Message.model_validate(data)
        except ValueError:return  # Existing serialized validation owns errors.
        intent=message.intent
        # Only operations whose existing policy always revokes old motion.
        # A stale-echo neutral/deadman=True packet is discarded by that policy;
        # stopping for it here would let the previous active input resume later.
        cancel=message.op=='stop' or (
            self.release_mode=='immediate' and message.op=='intent' and intent is not None and
            not intent.deadman)
        if not cancel:return
        with self.input_cancellation_lock:
            if owner!=self.owner or self.session is None or message.seq<=self.session.sequence:return
            pending=self.pending_input_cancellation
            # A connection-close marker outlives any in-flight input handler.
            if pending is not None and (pending[1] is None or pending[1]>=message.seq):return
            self.pending_input_cancellation=(owner,message.seq)

    def notify_disconnected(self,owner):
        with self.input_cancellation_lock:
            if self.owner==owner:self.pending_input_cancellation=(owner,None)

    def cancel_pending_input(self):
        with self.input_cancellation_lock:pending=self.pending_input_cancellation
        if pending is None or pending[0]!=self.owner:return False
        self.request_stop()
        return True

    def receive(self,data):
        with self.input_cancellation_lock:pending=self.pending_input_cancellation
        try:return self._receive(data)
        finally:
            with self.input_cancellation_lock:
                if pending is not None and pending[1]==data.get('seq') and self.pending_input_cancellation==pending:
                    self.pending_input_cancellation=None

    def _receive(self,data):
        if self.session is None:raise RuntimeError('No controller')
        if data.get('op')=='arm_enable':
            try:
                if not self.session.receive(data):return
                sample=self.reader.snapshot()
                reason=self.enable_unavailable(sample)
                if reason:raise ValueError(reason)
                # Mark before submission: uncertain writes are never retried.
                self.enable_phase='awaiting_reply';self.enable_started_ns=self.clock()
                self.enable_generation=sample['generation'];self.enable_reason=None
                self.session.ready=False;self.base=None;self.flange_base=None;self.target=None;self.reason=None
                self.idle_samples.clear();self.stationary=False
                self.enable_pending=self.payload.enable(self.robot)
                self.events.append(dict(event='enable_requested',at_ns=self.clock(),
                    payload=self.payload.description(),
                    pre_action_observation=sample,receipt=dict(self.enable_pending.event)))
            except (ValueError,RuntimeError) as exc:
                self.session.reason=str(exc)
                if self.enable_phase=='awaiting_reply' and self.enable_pending is None:
                    self.enable_phase='uncertain';self.enable_reason=str(exc)
                    self.events.append(dict(event='enable_uncertain',at_ns=self.clock(),reason=str(exc)))
            except Exception as exc:
                self.enable_phase='uncertain';self.enable_reason=str(exc);self.session.reason=str(exc)
                self.events.append(dict(event='enable_uncertain',at_ns=self.clock(),reason=str(exc)))
            return
        if data.get('op')=='camera_restart':
            if not self.session.receive(data):return
            reason=self.camera_restart_unavailable()
            if reason:
                self.session.reason=reason
                return
            self.suspend('相机重新连接；画面恢复后请重新准备')
            try:
                identity=self.camera_restart.request()
                self.events.append(dict(event='camera_restart_requested',at_ns=self.clock(),request_id=identity))
            except Exception as exc:self.session.reason='相机重连请求失败: '+str(exc)
            return
        if data.get('op') in ('record_start','record_finish'):
            # Recording is an explicit UI operation, not a periodic intent.
            # Preserve its rejection until a new request, so a later neutral
            # input cannot turn an ignored click into an apparent success.
            result=dict(operation=data['op'],sequence=data.get('seq'),
                        processed_ns=self.clock(),accepted=False,reason=None)
            try:
                if self.episodes is None:raise ValueError('原始录制未配置')
                if not self.session.receive(data):
                    result['reason']=self.session.reason or '本次录制请求未被接受（已过期或序号无效）；请等待状态更新后重新操作'
                    return
                if data['op']=='record_start':
                    reason=self.recording_health()
                    if reason or not self.stationary or self.moving or self.stop_pending:
                        raise ValueError(reason or '开始录制前请释放操作并等待静止')
                    self.episodes.request('start',task=data.get('task') or '',requested_ns=self.clock(),state=self.state(),
                                          research_context=data.get('research_context'))
                else:
                    self.episodes.request('finish',result=data.get('result'),reason=data.get('reason'))
                    self.suspend('recording_finished')
                self.session.reason=None
                result['accepted']=True
            except (ValueError,RuntimeError) as exc:
                self.session.reason=str(exc)
                result['reason']=str(exc)
            finally:
                self.recording_request=result
                self.events.append(dict(event='recording_request_result',at_ns=self.clock(),result=dict(result)))
            return
        if data.get('op')=='prepare' and self.episodes and self.episodes.state()['phase'] in ('starting','sealing'):
            self.session.reason='原始片段正在初始化或封存，请稍候'
            return
        if data.get('op')=='prepare' and self.gripper and self.gripper.snapshot()['phase'] in ('connecting','initializing'):
            self.session.reason='夹爪初始化期间保持机械臂静止'
            return
        if (data.get('op')=='prepare' and self.dynamic_gripper and self.gripper and
                self.gripper.snapshot()['phase']=='uninitialized'):
            self.session.reason='Waiting for automatic gripper connection'
            return
        if data.get('op')=='gripper_initialize':
            if self.enable_pending is not None or self.enable_phase=='observing':
                self.session.reason='使能完成并稳定观察后再初始化夹爪'
                return
            sample=self.reader.snapshot()
            reason=self.healthy(sample)
            if reason or not self.stationary or self.moving or self.stop_pending:
                self.session.reason=reason or '夹爪初始化前请释放操作并等待机械臂静止'
                return
            if self.session.receive(data):
                if self.gripper is None:
                    self.session.reason='影子模式不初始化CAN'
                else:
                    try:self.gripper.connect()
                    except Exception as exc:self.session.reason=str(exc)
            return
        if data.get('op')=='prepare':
            if self.session.ready:return
            if self.enable_pending is not None or self.enable_phase=='observing':
                self.session.reason='使能完成并稳定观察后再准备'
                return
            sample=self.reader.snapshot()
            reason=self.healthy(sample)
            if reason:
                self.session.reason=reason
                return
            if self.moveit and not self.moveit.snapshot()['ready']:
                self.session.reason='运动模型尚未就绪: '+str(self.moveit.snapshot().get('error'))
                return
            if self.moveit and self.tool.name!='flange' and self.moveit.snapshot().get('tcp')!=self.tool.description():
                self.session.reason='运动模型TCP与当前控制点不一致'
                return
            if self.assembly_id and self.moveit.snapshot().get('assembly_id')!=self.assembly_id:
                self.session.reason='Workspace assembly does not match the control model'
                return
            if self.stop_pending is not None:
                self.session.reason='等待停止确认'
                return
            if not self.stationary or not self.idle_samples or self.clock()-self.idle_samples[0][0]<5_000_000_000:
                self.session.reason='等待5秒稳定反馈'
                return
            if self.moveit:
                tracking_reason=self.joint_tracking.rebase(sample)
                if tracking_reason:
                    detail=self.joint_tracking.last_detail
                    self.session.reason=(f"J{detail['joint']}目标与实际偏差"
                        f"{max(detail['error_deg']):.2f}°，超过停止后准备容差"
                        f"{detail['limit_deg']:.1f}°；请检查接触受力")
                    return
            # Reconnection follows an explicit prepare and fresh device idle state.
            if self.stage=='live' and (self.dashboard.uncertain or not self.dashboard._connected):
                self._connect_dashboard()
            if self.session.receive(data):
                position,rotation=pose_of(sample)
                joints=np.deg2rad(sample['status']['joint_state']['q_actual'])
                if self.flange_base is None:
                    self.flange_base=(position.copy(),rotation,joints.copy())
                position,rotation=self.tool.apply(position,rotation)
                # Preparing resumes from fresh actual state, but must not move
                # the site's workspace origin after a stop or a reconnection.
                # Otherwise repeated prepare/retreat cycles walk the envelope
                # out of the cable-clearance region qualified for this session.
                if self.base is None:
                    self.base=(position.copy(),rotation,joints.copy())
                self.target=(position.copy(),rotation,joints.copy())
                self.generation=sample['generation']
                self.reason=None
                self.stop_failed=False
                self.blocked_signature=None
                self.pending=None
                self.moving=False
                self.trajectory=None
                self.braking=False
                self.brake_finished=False
                self.last_moveit_candidate_ns=None
                if self.moveit:self.moveit.stop()
                self.events.append(dict(event='prepared',at_ns=self.clock(),base=self.workspace_origin(),
                    start=dict(position_m=position.tolist(),quaternion_xyzw=rotation.as_quat().tolist(),
                               joints_rad=joints.tolist()),release_mode=self.release_mode))
            return
        accepted=False
        incoming=data.get('intent') or {}
        if incoming.get('orientation_assist') and (not self.moveit or self.moveit_api!='cpp' or
                self.moveit.snapshot().get('orientation_assist',{}).get('version')!=1):
            self.suspend('orientation_assist_unavailable')
            return
        try:
            accepted=self.session.receive(data)
        except Exception as exc:
            self.session.suspend('输入格式错误: '+str(exc))
        if self.session.stop_requested:
            if self.session.ready and self.session.input_paused_ns is None and self.release_mode=='ruckig':self.request_release()
            else:self.request_stop()
        elif accepted and data.get('op')=='intent':
            if self.moveit:self._prime_moveit()
            else:self._prime_native()

    def _prime_native(self):
        """Wake the idle control tick, never send from the input thread."""
        if (self.native_started or self.stop_failed or self.stop_pending is not None
                or self.enable_pending is not None or self.enable_phase=='observing'):
            return
        intent=self.session.active()
        if self._workspace_input_blocked(intent):return
        if intent is not None and any(intent.linear_m_s+intent.angular_rad_s+intent.joint_rad_s):
            self.native_wake.set()

    def _prime_moveit(self):
        """Start candidate computation on fresh input, without a device write.

        The 20 Hz tick still checks health and alone consumes/sends targets.
        Release clears the bridge epoch synchronously, including primed work.
        """
        if (self.moveit is None or self.stop_failed or self.stop_pending is not None
                or self.tracking_stop_receipt_ns is not None
                or self.enable_pending is not None or self.enable_phase=='observing'):
            return
        intent=self.session.active()
        if self.session.stop_requested:
            self.request_stop()
            return
        if self._workspace_input_blocked(intent):return
        if intent is None or not any(intent.linear_m_s+intent.angular_rad_s+intent.joint_rad_s):
            return
        sample=self.reader.snapshot()
        reason=self.healthy(sample)
        if reason or self.generation!=sample['generation']:
            self.suspend(reason or '反馈连接已重建，请重新准备')
            return
        if self.moveit.snapshot()['ready']:
            self.moveit.workspace=self.workspace.velocity_boundary(self.base[0])
            self.moveit.submit(intent,self.session.intent_sequence)

    def request_release(self):
        """Normal release only. Faults always bypass the trajectory and Stop.

        Both the first deceleration command and later Stop are recorded. This
        experiment does not relabel a late Stop as meeting the 50 ms criterion.
        """
        if self.gripper:self.gripper.hold()
        self.session.stop_requested=False
        if self.braking:return
        if self.trajectory is None or (not self.moving and self.stage!='shadow'):
            self.request_stop()
            return
        self.braking=True
        self.events.append(dict(event='release_deceleration_requested',at_ns=self.clock()))

    def request_stop(self):
        self.queued_moveit=None
        self.native_started=False
        self.native_wake.clear()
        if self.gripper:self.gripper.hold()
        if self.moveit:self.moveit.stop()
        self.last_moveit_candidate_ns=None
        if self.episodes and self.session and not self.session.ready and (self.session.reason or self.reason):
            # A centered, unprepared session may record observations. Neutral
            # input cancellation is not an operator Stop or a device fault.
            self.episodes.abort(self.session.reason or self.reason)
        if self.session:self.session.stop_requested=False
        self.braking=False
        self.brake_finished=False
        self.trajectory=None
        self.target=None
        if self.stage=='shadow' or not self.moving:
            return
        if self.stop_pending is None and not self.stop_failed:
            self.stop_acknowledged=False
            self.stationary=False
            try:
                self.stop_pending=self.robot.Stop()
                # The receiver updates Pending.event when the reply arrives.
                # Both journals must keep the same request-time observation.
                self.events.append(dict(event='stop_requested',at_ns=self.clock(),receipt=dict(self.stop_pending.event)))
            except Exception as exc:
                self.stop_failed=True
                self.reason='停止发送异常: '+str(exc)
                if self.session:self.session.suspend(self.reason)

    def suspend(self,reason):
        self.reason=reason
        if self.session:self.session.suspend(reason)
        self.request_stop()

    def _pause_direction(self,reason):
        """Reject this stream without revoking the operator's prepared session."""
        first=self.session.input_paused_ns is None
        self.session.pause_input(reason,allow_direction_change=True)
        if first:
            self.events.append(dict(event='control_direction_limited',at_ns=self.clock(),
                reason=reason,prepared=self.session.ready,
                blocked_direction=self.session.paused_direction))
        self.reason=None
        self.request_stop()

    def _settle_stopped_tracking(self,sample):
        """Retire a cancelled endpoint only after acknowledged, observed Stop.

        Neutral input or a solver reset is not evidence of a physical stop.
        Keep the old guard until a newer healthy frame and the existing idle
        window prove that no command remains outstanding. The next writer
        then uses the strict initial-step bound from the stopped actual pose.
        """
        receipt=self.tracking_stop_receipt_ns
        if receipt is None:
            if (self.moveit and self.session and self.session.ready and self.joint_tracking.last_sent is None
                    and self.stationary and not self.moving and self.pending is None
                    and self.stop_pending is None and not self.stop_failed
                    and not self.healthy(sample) and self.generation==sample['generation']):
                # No arm write has occurred since confirmed idle/Prepare.
                # Unloading a gripped object may change actual while QTarget
                # stays fixed; refresh the idle origin before the first write.
                return self.joint_tracking.rebase(sample) is None
            return True
        if (self.stop_failed or not self.stop_acknowledged or self.stop_pending is not None
                or self.pending is not None or self.moving or not self.stationary
                or sample['received_ns']<=receipt or self.healthy(sample)
                or self.generation!=sample['generation']):
            return False
        if self.joint_tracking.rebase(sample):return False
        self.tracking_stop_receipt_ns=None
        self.events.append(dict(event='joint_tracking_rebased',at_ns=self.clock(),
                                stop_receipt_ns=receipt,pre_action_observation=sample))
        return True

    def tick(self):
        sample=self.reader.snapshot()
        now=self.clock()
        dt=self.servo_period_s if self.last_tick_ns is None else min(self.servo_period_s,max(0,(now-self.last_tick_ns)/1e9))
        self.last_tick_ns=now
        self.last_snapshot=sample
        # Observation and health checks precede all command-completion paths.
        self._observe_stationary(sample)
        if self.dynamic_gripper and self.moveit and self.stationary and not self.moving and self.pending is None and self.stop_pending is None:
            with self.moveit.geometry_lock:self.moveit.last_arm_target=None
        self._advance_enable(sample)
        if (self.gripper and self.gripper.snapshot()['phase']=='uninitialized' and self.stationary and
                not self.moving and self.pending is None and self.stop_pending is None and
                self.enable_pending is None and self.enable_phase!='observing' and not self.healthy(sample)):
            # One controller-owned feedback-only attachment, independent of
            # browser refresh/tab switches. Never retries a fault or prepares.
            self.gripper.connect()
        if self.gripper and self.gripper.snapshot()['phase'] in ('connecting','initializing','ready'):
            grip_reason=self.healthy(sample)
            if grip_reason:
                if self.gripper.snapshot()['phase'] in ('connecting','initializing'):
                    # No established target yet: retain failed initialization.
                    self.gripper.fault(grip_reason)
                else:
                    # Arm/video readiness is not a CAN fault. Freeze the last
                    # submitted gripper target and suspend the control session;
                    # fresh health alone cannot resume input. Own CAN faults
                    # remain latched by the independent gripper worker.
                    self.suspend(grip_reason)
        if self.session is not None and (self.session.ready or self.moving):
            reason=self.healthy(sample)
            if reason or (self.generation is not None and sample['generation']!=self.generation):
                self.suspend(reason or '反馈连接已重建，请重新准备')
        if (self.moveit and self.moving and self.session and self.session.ready and
                self.stop_pending is None and not self.healthy(sample)):
            tracking_reason=self.joint_tracking.observe(sample,now)
            if tracking_reason:
                self.events.append(dict(event='joint_tracking_stopped',at_ns=now,
                    detail=dict(self.joint_tracking.last_detail),pre_action_observation=sample))
                self.suspend('关节跟随异常：已停止，请检查关节实际运动')
                return
        if self.stop_pending is not None:
            try:
                if self.dashboard.poll(self.stop_pending):
                    self.last_receipt=dict(self.stop_pending.event)
                    self.events.append(dict(event='stop_result',at_ns=self.clock(),receipt=self.last_receipt))
                    self.stop_acknowledged=self.stop_pending.event['accepted']
                    if not self.stop_acknowledged:
                        self.stop_failed=True
                        self.reason='控制器未接受停止'
                        if self.session:self.session.suspend(self.reason)
                    self.stop_pending=None
                    if self.stop_acknowledged:
                        self.moving=False
                        if self.moveit:self.tracking_stop_receipt_ns=self.clock()
            except Exception as exc:
                self.reason='停止回包不确定: '+str(exc)
                if self.session:self.session.suspend(self.reason)
                self.last_receipt=dict(self.stop_pending.event)
                self.events.append(dict(event='stop_result',at_ns=self.clock(),receipt=self.last_receipt))
                self.stop_pending=None
                self.stop_failed=True
        if self.pending is not None:
            try:
                if self.dashboard.poll(self.pending):
                    self.last_receipt=dict(self.pending.event)
                    if not self.pending.event['accepted']:
                        self.pending=None
                        self.suspend('控制器拒绝目标；可重新准备并调整方向')
                        return
                    self.accepted=dict(self.candidate,receipt=dict(self.pending.event))
                    self.events.append(dict(event='motion_confirmed',at_ns=self.clock(),target=self.accepted))
                    self.pending=None
            except Exception as exc:
                self.events.append(dict(event='motion_uncertain',at_ns=self.clock(),receipt=dict(self.pending.event)))
                self.pending=None
                self.suspend('动作确认不确定: '+str(exc))
                return
        if self.session is None:return
        if self.cancel_pending_input():return
        if self.session.input_paused_ns is not None and self.moveit:
            info=self.moveit.snapshot()
            if not info['ready']:
                self.suspend('运动模型不可用: '+str(info.get('error')))
                return
        if not self._settle_stopped_tracking(sample):
            if self.moveit and not self.moveit.snapshot()['ready']:
                self.suspend('运动模型不可用: '+str(self.moveit.snapshot().get('error')))
            return
        intent=self.session.active()
        workspace_held=self._workspace_input_blocked(intent)
        if self.gripper and (intent is None or not intent.gripper_rate_s):self.gripper.hold()
        if self.session.stop_requested:self.request_stop()
        if self.braking and (not self.session.ready or now-self.session.received_ns>250_000_000):
            self.suspend('减速期间输入连接过期')
        if intent is None and not self.braking:
            if self.moveit:self.moveit.stop()
            return
        if self.gripper and self.gripper.snapshot()['phase'] in ('connecting','initializing'):
            if self.moving:self.request_stop()
            elif self.moveit:self.moveit.stop()
            return
        if (self.release_mode=='ruckig' and self.trajectory is not None and intent is not None
                and not any(intent.linear_m_s+intent.angular_rad_s+intent.joint_rad_s)):
            self.request_release()
        if intent is not None and self.gripper and intent.gripper_rate_s:
            grip=self.gripper.snapshot()
            if grip['phase']=='ready':
                if self.cancel_pending_input():return
                self.gripper.set_stroke_rate(intent.gripper_rate_s,min(dt,.1))
            # A gripper-only input does not enqueue an unrelated arm hold.
            if not any(intent.linear_m_s+intent.angular_rad_s+intent.joint_rad_s) and not self.braking:
                if self.moving:self.request_stop()
                elif self.moveit:self.moveit.stop()
                return
        if self.stop_failed or self.stop_pending is not None:return
        if workspace_held:return
        if self.moveit:
            if not any(intent.linear_m_s+intent.angular_rad_s+intent.joint_rad_s):
                if self.moving:self.request_stop()
                else:self.moveit.stop()
                return
            info=self.moveit.snapshot()
            if not info['ready'] or (self.tool.name!='flange' and info.get('tcp')!=self.tool.description()):
                self.suspend('运动模型不可用: '+str(info.get('error')))
                return
            self.moveit.workspace=self.workspace.velocity_boundary(self.base[0])
            self.moveit.submit(intent,self.session.intent_sequence)
        if self.pending is not None:return
        if self.moveit:
            self._moveit_step(sample)
            return
        if self.brake_finished:
            self.request_stop()
            return
        actual_position,actual_rotation=pose_of(sample)
        actual_joints=np.deg2rad(sample['status']['joint_state']['q_actual'])
        if self.target is None:self.target=(actual_position,actual_rotation,actual_joints)
        position,rotation,joints=self.target
        if self.release_mode=='ruckig' or self.native_smoothing=='ruckig':
            from .trajectory import VelocityTrajectory
            kind=self.trajectory.kind if self.braking else ('joint' if any(intent.joint_rad_s) else 'cartesian')
            requested=np.zeros(6)
            if not self.braking:
                if kind=='joint':requested=np.array(intent.joint_rad_s)
                else:
                    requested=np.array(intent.linear_m_s+intent.angular_rad_s)
                    if intent.frame=='tool':
                        requested=np.concatenate((rotation.apply(requested[:3]),rotation.apply(requested[3:])))
            if self.native_smoothing=='ruckig' and not np.any(requested):
                # Centered axes are a normal immediate Stop, including when
                # deadman stays pressed. Do not send a deceleration tail.
                self.request_stop()
                return
            if self.trajectory is None:
                if not np.any(requested):return
                self.trajectory=VelocityTrajectory(self.servo_period_s,kind,
                    motion_profile=self.motion_profile.name if self.native_smoothing=='ruckig' else None)
            if self.trajectory.kind!=kind:
                # Mode changes cannot discard an active velocity reference.
                if not self.trajectory.stopped:
                    if self.native_smoothing=='ruckig':self.request_stop()
                    else:self.request_release()
                    return
                self.trajectory=VelocityTrajectory(self.servo_period_s,kind,
                    motion_profile=self.motion_profile.name if self.native_smoothing=='ruckig' else None)
            if not np.any(requested) and self.trajectory.stopped and not self.braking:return
            delta=self.trajectory.step(requested)
            if kind=='joint':
                new_joints=joints+delta
                new_position,new_rotation=actual_position,actual_rotation
            else:
                new_position=position+delta[:3]
                new_rotation=Rotation.from_rotvec(delta[3:])*rotation
                new_joints=actual_joints
            self.brake_finished=self.braking and self.trajectory.stopped
        elif any(intent.joint_rad_s):
            if not self.native_started:dt=self.servo_period_s
            new_joints=joints+np.array(intent.joint_rad_s)*dt
            new_position,new_rotation=actual_position,actual_rotation
            kind='joint'
        else:
            # Neutral deadman is intent readiness, not a motion command. Native
            # zero-increment qualification belongs to the explicit probe.
            if not any(intent.linear_m_s+intent.angular_rad_s):
                if self.moving:self.request_stop()
                return
            # An idle-start wake can be sooner than the previous idle tick.
            # The first target still spans exactly one SDK command period,
            # measured from current actual state, with no old input catch-up.
            if not self.native_started:dt=self.servo_period_s
            delta=np.array(intent.linear_m_s)*dt
            dr=Rotation.from_rotvec(np.array(intent.angular_rad_s)*dt)
            if intent.frame=='tool':delta=rotation.apply(delta)
            new_rotation=rotation*dr if intent.frame=='tool' else dr*rotation
            new_position=position+delta
            new_joints=actual_joints
            kind='cartesian'
        if self.cancel_pending_input():return
        workspace=evaluate_workspace(self.base,(actual_position,actual_rotation,actual_joints),
                                     (new_position,new_rotation,new_joints),self.workspace)
        if self.cancel_pending_input():return
        if self._reject_workspace(workspace,sample):return
        self.native_started=True
        self.reason=None
        self.target=(new_position,new_rotation,new_joints)
        self.candidate=dict(kind=kind,position_m=new_position.tolist(),
            quaternion_xyzw=new_rotation.as_quat().tolist(),
            joints_rad=new_joints.tolist() if kind=='joint' else None,
            observed_joints_rad=actual_joints.tolist(),
            source_input=dict(sequence=self.session.intent_sequence,received_ns=self.session.received_ns,
                              intent=self.session.intent.model_dump() if self.session.intent is not None else None),
            input_sequence=self.session.intent_sequence,pre_action_received_ns=sample['received_ns'],
            trajectory=None if self.trajectory is None else self.trajectory.state(),braking=self.braking,
            workspace=workspace)
        if self.stage=='shadow':return
        if self.cancel_pending_input():return
        self.moving=True
        if kind=='joint':
            self.pending=self.motion.ServoJ(np.rad2deg(new_joints),t=self.servo_duration_s,aheadtime=self.servo_aheadtime,gain=self.servo_gain)
        else:
            self.pending=self.motion.ServoP([*(new_position*1000),*new_rotation.as_euler('xyz',degrees=True)],
                                          t=self.servo_duration_s,aheadtime=self.servo_aheadtime,gain=self.servo_gain)
        self.events.append(dict(event='motion_submitted',at_ns=self.clock(),candidate=self.candidate,
                                pre_action_observation=sample,receipt=dict(self.pending.event)))
        self.last_native_sent_ns=self.clock()

    def _moveit_step(self,sample):
        if self.queued_moveit is not None:
            self._dispatch_moveit(sample)
            return
        # Prepare one endpoint ahead of its dispatch slot. The producer waits
        # for our actual write receipt, so no intermediate point is skipped.
        row=self.moveit.take()
        if self.cancel_pending_input():return
        blocked={2:'该方向接近奇异位置：松开后可在当前模式反向退回',5:'模型碰撞边界：当前方向已停止',6:'关节边界：可向范围内退回'}
        solver=self.moveit.snapshot();status=solver.get('status')
        rejection=solver.get('last_rejection_detail') or {}
        rejection_id=rejection.get('id',0)
        if rejection_id>self.last_solver_rejection_id:
            self.last_solver_rejection_id=rejection_id
            if rejection.get('reason')!='released_or_expired_input':
                # Latest-only IPC may already show a subsequent neutral status.
                # An unseen rejection is a writer barrier even in that case.
                self.events.append(dict(event='solver_rejection_barrier',at_ns=self.clock(),detail=rejection))
                self._pause_direction('solver_direction_blocked')
                return
        if status not in blocked and row is not None:status=row.get('status')
        if status in blocked:
            # Servo can deliberately stop publishing trajectories at a bound.
            # This is a normal directional block, not lost IPC. Discard any
            # residual rolling target and Stop the moving device once. Do not
            # repeatedly reset a stationary bridge: that would prevent a new
            # retreat command/type from reaching Servo and clearing its status.
            reason=blocked[status]
            assist=solver.get('orientation_assist') or {}
            if status==6 and assist.get('enabled') and assist.get('reason')=='final_angular_envelope':
                reason='姿态辅助角度边界：松开后可重新操作或手动转腕'
            check=solver.get('gripper_check') or {}
            if status==5 and not check.get('arm_safe',True):
                reason={'sweep_travel_limit':'目标与反馈间扫掠过大：松开后可重新操作',
                        'sweep_budget':'几何检查超时：当前方向已停止',
                        'callback_deadline':'几何检查超时：当前方向已停止'}.get(check.get('arm_reason'),reason)
            if self.reason!=reason:
                self.events.append(dict(event='model_direction_blocked',at_ns=self.clock(),
                                        status=status,reason=reason,gripper_check=check,
                                        orientation_assist=assist,
                                        collision_retreat=solver.get('collision_retreat'),callback_us=solver.get('callback_us')))
            self.reason=reason;self.candidate=None;self.target=None
            if self.moving:self.request_stop()
            return
        if row is None:
            take=getattr(self.moveit,'last_take',None) or {}
            if (solver.get('writer_pacing_version')==1 and take.get('reason') not in
                    (None,'no_candidate','already_consumed','accepted','inactive_epoch')):
                # No physical receipt can arrive for a discarded endpoint.
                # Cancel its epoch even if this was the first point and the
                # robot never started, rather than waiting forever for it.
                self._pause_dispatch('candidate_'+take['reason'],take)
                return
            from .delivery_timing import stream_timeout_ns
            stream_limit=stream_timeout_ns(solver)
            if self.moving and self.last_moveit_candidate_ns is not None and self.clock()-self.last_moveit_candidate_ns>stream_limit:
                self.events.append(dict(event='model_stream_paused',at_ns=self.clock(),
                    candidate_gap_ms=(self.clock()-self.last_moveit_candidate_ns)/1e6,
                    stream_timeout_ms=stream_limit/1e6,model=solver))
                # Use the pinned stream grace and discard all pending intent
                # and trajectory work. A transient computation gap is not a
                # device/session failure: accept a fresh direction change or
                # neutral instead of aborting the recording.
                # Health failures, worker failure and failed Stop still disarm.
                self._pause_direction('solver_stream_stale')
            return
        self.last_moveit_candidate_ns=self.clock()
        fp=np.array(row['position_m']);fr=Rotation.from_quat(row['quaternion_xyzw'])
        position,rotation=self.tool.apply(fp,fr)
        joints=np.array(row['joints_rad'])
        if not np.all(np.isfinite(np.concatenate((position,joints)))):raise ValueError('MoveIt candidate nonfinite')
        afp,afr=pose_of(sample)
        ap,ar=self.tool.apply(afp,afr)
        actual=np.deg2rad(sample['status']['joint_state']['q_actual'])
        if self.cancel_pending_input():return
        workspace=evaluate_workspace(self.base,(ap,ar,actual),(position,rotation,joints),self.workspace)
        if self.tool.name!='flange' and self.workspace.mode=='local' and self.workspace.rotation_deg is not None:
            flange_workspace=evaluate_workspace(self.flange_base,(afp,afr,actual),(fp,fr,joints),self.workspace)
            workspace['flange']=flange_workspace
            workspace['allowed']=workspace['allowed'] and flange_workspace['allowed']
        elif self.tool.name!='flange':
            # At a fixed TCP, rotating the tool necessarily translates the
            # flange. Bound the TCP position and absolute joints, not the
            # compensation displacement against the old flange sphere.
            workspace['flange']=dict(policy='tcp_rotation_compensation',
                position_delta_m=float(np.linalg.norm(fp-self.flange_base[0])))
        if self.joint_range is not None:
            joint_range=self.joint_range.evaluate(actual,joints)
            workspace['absolute_joints']=joint_range
            workspace['allowed']=workspace['allowed'] and joint_range['allowed']
        if self.cancel_pending_input():return
        if self._reject_workspace(workspace,sample):return
        self.reason=None
        self.target=(position,rotation,joints)
        self.candidate=dict(kind='moveit_servoj',position_m=position.tolist(),quaternion_xyzw=rotation.as_quat().tolist(),
            pose_frame=self.tool.frame,tcp=self.tool.description(),
            flange_pose=dict(position_m=fp.tolist(),quaternion_xyzw=fr.as_quat().tolist(),pose_frame='controller_flange'),
            joints_rad=joints.tolist(),input_sequence=row['input_sequence'],moveit=row,
            pre_action_received_ns=sample['received_ns'],workspace=workspace)
        if self.stage=='shadow':return
        # A zero result from singularity/collision/limit handling is a normal
        # directional block. Keep the prepared session and allow retreat.
        if np.max(np.abs(joints-actual))<1e-7:
            if solver.get('writer_pacing_version')==1:
                self._pause_direction('solver_direction_blocked')
            elif self.moving:self.request_stop()
            return
        if solver.get('writer_pacing_version')==1 and (row.get('dispatch') or {}).get('version')!=1:
            raise ValueError('Missing writer-paced candidate provenance')
        self.queued_moveit=self.candidate
        self._dispatch_moveit(sample)

    def scheduled_write_ns(self):
        if self.queued_moveit is None:return None
        dispatch=self.queued_moveit['moveit'].get('dispatch')
        if not dispatch:return None  # Legacy/ROS routes retain their scheduler.
        return max(dispatch['not_before_ns'],
                   (self.last_moveit_sent_ns+round(self.servo_period_s*1e9)) if self.last_moveit_sent_ns is not None else 0)

    def _pause_dispatch(self,reason,row):
        self.events.append(dict(event='motion_dispatch_rejected',at_ns=self.clock(),reason=reason,
                                id=row.get('id'),epoch=row.get('epoch')))
        self._pause_direction('solver_stream_stale')

    def _dispatch_moveit(self,sample):
        candidate=self.queued_moveit
        if candidate is None:return
        row=candidate['moveit'];joints=np.array(candidate['joints_rad'])
        due=self.scheduled_write_ns()
        if due is not None and self.clock()<due:return
        from .delivery_timing import candidate_deadline_ns
        deadline=candidate_deadline_ns(row)
        if self.clock()>=deadline:
            self._pause_dispatch('original_candidate_deadline',row)
            return
        if row.get('dispatch'):
            origin=self.moveit.dispatch_snapshot()['receipt']
            parent=row['cpp']['parent_id']
            if (row['epoch']!=self.moveit.epoch or
                    (parent is None and origin and origin['epoch']==row['epoch']) or
                    (parent is not None and (not origin or
                     (origin['epoch'],origin['id'])!=(row['epoch'],parent) or row['id']<=parent))):
                self._pause_dispatch('uncommitted_solver_parent',row)
                return
        if self.cancel_pending_input():return
        from contextlib import nullcontext
        with self.moveit.geometry_lock if self.dynamic_gripper else nullcontext():
            # Recheck at the actual writer boundary, after candidate selection.
            # A concurrent CAN commitment must not slip past an older sweep.
            if self.dynamic_gripper and not self.moveit.gripper_allows_arm(row):
                if row.get('dispatch'):self._pause_dispatch('gripper_path_changed',row)
                else:self.queued_moveit=None
                return
            writer_sample=self.reader.snapshot()
            writer_now=self.clock()
            reason=self.healthy(writer_sample)
            if reason:
                self.suspend(reason)
                return
            tracking_reason=self.joint_tracking.observe(writer_sample,writer_now)
            if tracking_reason is None:
                tracking_reason=self.joint_tracking.candidate(np.rad2deg(joints),writer_sample,writer_now)
            if tracking_reason:
                self.events.append(dict(event='joint_tracking_stopped',at_ns=writer_now,
                    detail=dict(self.joint_tracking.last_detail),candidate=self.candidate,
                    pre_action_observation=writer_sample))
                if tracking_reason=='joint_command_discontinuity':
                    self._pause_direction('solver_target_discontinuity')
                else:
                    self.suspend('关节目标不连续或跟随异常：已停止，请检查后重新准备')
                return
            if self.clock()>=deadline:
                self._pause_dispatch('original_candidate_deadline',row)
                return
            try:
                with self.command_port.deadline(deadline) if self.command_port else nullcontext():
                    self.pending=self.motion.ServoJ(np.rad2deg(joints),t=self.servo_duration_s,
                                                   aheadtime=self.servo_aheadtime,gain=self.servo_gain)
            except MotionDeadlineExpired:
                self._pause_dispatch('socket_candidate_deadline',row)
                return
            self.moving=True
            self.joint_tracking.committed(np.rad2deg(joints),self.pending.event.get('sent_ns',self.clock()))
            if self.dynamic_gripper:self.moveit.last_arm_target=joints.tolist()
        self.last_moveit_sent_ns=self.pending.event.get('sent_ns',self.clock())
        if row.get('dispatch'):self.moveit.committed(row,self.last_moveit_sent_ns)
        self.queued_moveit=None
        self.events.append(dict(event='motion_submitted',at_ns=self.clock(),candidate=self.candidate,
            pre_action_observation=sample,receipt=dict(self.pending.event)))

    @staticmethod
    def _motion_direction(intent):
        if intent is None or not intent.deadman:return None
        values=intent.linear_m_s+intent.angular_rad_s+intent.joint_rad_s
        if not any(values):return None
        # Input already passed the mapper's dead zone. A speed-slider change
        # must not rearm a held outward command after a boundary Stop.
        return (intent.frame,tuple(1 if v>0 else -1 if v<0 else 0 for v in values))

    def _workspace_input_blocked(self,intent):
        direction=self._motion_direction(intent)
        if direction!=self.blocked_signature:
            self.blocked_signature=None
        return direction is not None and direction==self.blocked_signature

    def _reject_workspace(self,workspace,sample):
        if workspace['allowed']:return False
        # Preserve the rejected candidate metrics, not only the last accepted
        # target. Log the transition once; holding outward must not flood I/O.
        reason=('关节限位余量：可向范围内退回' if not workspace.get('absolute_joints',{}).get('allowed',True)
                else '当前测试边界：可向范围内退回')
        if self.reason!=reason:
            self.events.append(dict(event='workspace_blocked',at_ns=self.clock(),
                input_sequence=self.session.intent_sequence,
                pre_action_received_ns=sample['received_ns'],workspace=workspace))
        self.reason=reason
        self.blocked_signature=self._motion_direction(self.session.active())
        self.request_stop()
        return True

    def _observe_stationary(self,sample):
        s=sample['status']
        now=self.clock()
        valid=(s is not None and sample['connected'] and sample['age_ms'] is not None
               and 0<=sample['age_ms']<=100 and s['robot_mode'] in (4,5)
               and not s['run_queued_cmd'])
        if not valid:
            self.idle_samples.clear()
            self.idle_signature=None
            self.stationary=False
            return
        q=tuple(s['joint_state']['q_actual'])
        signature=(sample['generation'],s['robot_mode'],s['enable_status'],
                   self.payload.mismatch(s) is None)
        if signature!=self.idle_signature:
            self.idle_samples.clear();self.idle_signature=signature
        if self.idle_samples and (self.idle_samples[-1][2]!=sample['generation']
                or any(abs(a-b)>.05 for a,b in zip(self.idle_samples[0][1],q))):
            self.idle_samples.clear()
        self.idle_samples.append((now,q,sample['generation']))
        while len(self.idle_samples)>1 and now-self.idle_samples[1][0]>5_100_000_000:
            self.idle_samples.popleft()
        window=[row[1] for row in self.idle_samples if now-row[0]<=300_000_000]
        # QDActual is retained verbatim but its observed stationary noise is not
        # a verified velocity bound. Stop evidence uses actual position history.
        self.stationary=bool(now-self.idle_samples[0][0]>=250_000_000
                             and all(max(axis)-min(axis)<=.01 for axis in zip(*window)))

    def enable_unavailable(self,sample):
        if self.stage!='live':return '影子模式不使能机械臂'
        if self.enable_phase!='idle':return '本次使能已请求；不会重复发送'
        reason=self.healthy(sample,require_enabled=False)
        if reason:return reason
        s=sample['status']
        if s['enable_status']:return '机械臂已使能，无需重复使能'
        if s['robot_mode']!=4 or s['run_queued_cmd']:return '等待机械臂进入未使能空闲状态'
        if not self.stationary or self.moving or self.pending or self.stop_pending or self.braking:
            return '使能前请释放操作并等待静止'
        if self.dashboard is None or not self.dashboard._connected or self.dashboard.uncertain:
            return '控制连接不可用；请先结束并重建连接'
        if self.episodes and self.episodes.state()['phase'] not in ('idle','complete','aborted'):
            return '片段正在录制或回传，暂不可使能'
        return None

    def _advance_enable(self,sample):
        if self.enable_pending is not None:
            try:
                if self.dashboard.poll(self.enable_pending):
                    self.enable_receipt=dict(self.enable_pending.event)
                    self.enable_phase='observing' if self.enable_receipt['accepted'] else 'rejected'
                    self.enable_reason=None if self.enable_receipt['accepted'] else '控制器拒绝使能'
                    self.events.append(dict(event='enable_result',at_ns=self.clock(),receipt=self.enable_receipt))
                    self.enable_pending=None
                    # Always collect a new five-second window after reply.
                    self.idle_samples.clear();self.stationary=False
            except Exception as exc:
                self.enable_receipt=dict(self.enable_pending.event)
                self.enable_phase='uncertain';self.enable_reason='使能回包不确定: '+str(exc)
                self.events.append(dict(event='enable_result',at_ns=self.clock(),receipt=self.enable_receipt,
                                        reason=self.enable_reason))
                self.enable_pending=None
        if self.enable_phase=='observing':
            reason=self.healthy(sample,require_enabled=False)
            if reason or sample['generation']!=self.enable_generation:
                self.enable_phase='failed';self.enable_reason=reason or '使能期间反馈连接重建'
            elif (sample['status']['enable_status'] and self.payload.mismatch(sample['status']) is None
                  and self.stationary and self.idle_samples and
                  self.clock()-self.idle_samples[0][0]>=5_000_000_000):
                self.enable_phase='complete';self.enable_reason=None
                if self.session and self.session.reason in ('使能完成并稳定观察后再准备','使能完成并稳定观察后再初始化夹爪'):
                    self.session.reason=None
                self.events.append(dict(event='enable_observed',at_ns=self.clock(),observation=sample))
            elif self.clock()-self.enable_started_ns>20_000_000_000:
                self.enable_phase='failed'
                self.enable_reason=self.payload.mismatch(sample['status']) or '使能后未取得稳定使能回读'

    def workspace_origin(self):
        if self.base is None:return None
        position,rotation,joints=self.base
        return dict(position_m=position.tolist(),quaternion_xyzw=rotation.as_quat().tolist(),
                    joints_rad=joints.tolist(),pose_frame=self.tool.frame,tcp=self.tool.description())

    def state(self,controller=False):
        # Retain the synchronous API for callers already holding the executor
        # lock. Runtime publications use observe_executor to render outside it.
        return render_observation(self.capture_state(controller))

    def capture_state(self,controller=False):
        sample=self.last_snapshot or self.reader.snapshot()
        camera=self.camera()
        gripper=self.gripper.snapshot() if self.gripper else dict(phase='uninitialized',actual=None,valid=False)
        captured_ns=self.clock()
        session=self.session
        recording=self.episodes.state() if self.episodes else dict(phase='unavailable')
        if self.recording_request is not None:
            recording['request']=dict(self.recording_request)
            if not self.recording_request['accepted']:
                # A recorder's actual failure remains the primary diagnosis.
                recording['error']=recording.get('error') or self.recording_request['reason']
        ready=bool(session and session.ready)
        # A directional model warning belongs to a prepared control session.
        # Once the operator stops or preparation fails, show that current
        # lifecycle reason rather than a stale warning from the last motion.
        reason=(session.reason if session and (not ready or session.input_paused_ns is not None) and session.reason
                else self.reason or (session.reason if session else None))
        direction_limited=bool(ready and session.input_paused_ns is not None and session.paused_direction is not None)
        if direction_limited:
            reason={
                'solver_direction_blocked':'当前方向受限',
                'solver_stream_stale':'计算或目标传递暂缓',
                'solver_target_discontinuity':'不连续目标已拒绝',
            }.get(session.reason,session.reason)+'；控制保持，可反向、回中或单独操作夹爪'
        feedback_valid=bool(sample.get('status') and sample.get('connected') and sample.get('age_ms') is not None
                            and 0<=sample['age_ms']<=100)
        model=self.moveit.snapshot() if self.moveit else None
        data=dict(kind='state',state='可控制' if direction_limited else '暂挂' if session and session.input_paused_ns is not None else '可控制' if ready else '暂挂' if reason else '可准备',
            ready=ready,reason=reason,input_paused=bool(session and session.input_paused_ns is not None),
            direction_limited=direction_limited,
            arm=dict(enabled=bool(sample['status']['enable_status']) if feedback_valid else None,
                mode=sample['status']['robot_mode'] if feedback_valid else None,
                enable_phase=self.enable_phase,enable_reason=self.enable_reason,
                payload_matches=(self.payload.mismatch(sample['status']) is None) if feedback_valid else None,
                can_enable=self.enable_unavailable(sample) is None,enable_receipt=self.enable_receipt,
                stable_ms=min(5000,max(0,(self.clock()-self.idle_samples[0][0])/1e6)) if self.idle_samples else 0),
            echo_ns=session.clock_tick() if session and controller else time.perf_counter_ns(),
            feedback=sample,camera=camera,gripper=gripper,candidate=self.candidate,
            workspace_origin=self.workspace_origin(),
            camera_restart=dict(self.camera_restart.status(),can_restart=self.camera_restart_unavailable() is None)
                if self.camera_restart else dict(available=False,phase='unavailable',can_restart=False),
            accepted=self.accepted,receipt=self.last_receipt,stage=self.stage,
            model=model,
            pending=None if self.pending is None else dict(self.pending.event),
            stop_acknowledged=self.stop_acknowledged,stationary=self.stationary,
            stopping=self.stop_pending is not None or self.braking,stop_uncertain=self.stop_failed,
            recording=recording,
            capabilities=dict(enabled=True,backend='E6 '+self.stage,
                              orientation_assist=dict(version=1,max_angle_deg=15,max_rate_rad_s=math.radians(3))
                                if model and self.moveit_api=='cpp' and
                                model.get('orientation_assist',{}).get('version')==1 else None,
                              gripper_input_units='usable_stroke_fraction_per_second',
                              gripper_feedback_units='native_can_fraction',gripper_stroke=gripper.get('stroke'),
                              memory_residency=getattr(self,'memory_residency',{'mode':'off','locked_bytes':0}),
                              workspace=dict(mode=self.workspace.mode,assembly_id=self.assembly_id,
                                             environment_modeled=False,restriction=reason),
                              joint_range=self.joint_range.description() if self.joint_range else None,
                              synthetic_keys=self.stage=='shadow' or self.acceptance_inputs,
                              servo_period_s=self.servo_period_s,
                              servo_duration_s=self.servo_duration_s,
                              servo_aheadtime=self.servo_aheadtime,servo_gain=self.servo_gain,
                              release_mode=self.release_mode,
                              native_smoothing=self.native_smoothing,
                              motion_engine='moveit_servo' if self.moveit else 'native_servop',
                              moveit_api=self.moveit_api,
                              original_recording=self.episodes is not None,
                              video_url='http://127.0.0.1:8889/gemini?muted=true&controls=false'+
                                ('&measure=1' if camera.get('latency_marker') else '')))
        return StateObservation(data,self.tool,self.motion_profile,self.payload,self.workspace,
                                self.limits,captured_ns)

    def release(self,owner):
        if self.owner==owner:
            self.suspend('控制连接关闭')
            with self.input_cancellation_lock:
                self.owner=None
                self.session=None
                self.pending_input_cancellation=None

    def drain_events(self):
        events=list(self.events)
        self.events.clear()
        return events

    def close(self):
        try:
            self.suspend('服务退出')
            # An already-sent Enable cannot be undone by closing its socket.
            # Drain its one official receipt; never compensate with Disable.
            deadline=time.monotonic()+(10.5 if self.enable_pending is not None else 2)
            while (self.stop_pending is not None or self.enable_pending is not None) and time.monotonic()<deadline:
                self.tick()
                time.sleep(.01)
            if self.enable_pending is not None:
                self.enable_receipt=dict(self.enable_pending.event)
                self.enable_phase='uncertain';self.enable_reason='退出时使能回包未确认'
                self.events.append(dict(event='enable_result',at_ns=self.clock(),receipt=self.enable_receipt,
                                        reason=self.enable_reason))
                self.enable_pending=None
        finally:
            try:
                if self.gripper:self.gripper.close()
            finally:
                try:
                    if self.dashboard is not None:self.dashboard.disconnect()
                finally:
                    try:
                        if self.moveit:self.moveit.close()
                    finally:
                        try:self.reader.close()
                        finally:
                            if self.camera_io:self.camera_io.close()


async def run(args):
    from .session_runner import servo_duration_option
    servo_duration_option(vars(args))  # Direct gateway entry must match the launcher.
    import fcntl
    lock=open('/run/e6-phosphobot-control.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    executor=NativeExecutor(args.host,args.stage,args.camera_health,acceptance_inputs=args.acceptance_inputs,
                            servo_period_s=args.servo_period_s,servo_duration_s=args.servo_duration_s,
                            servo_aheadtime=args.servo_aheadtime,servo_gain=args.servo_gain,
                            release_mode=args.release_mode,moveit_bundle=args.moveit_bundle,
                            moveit_api=getattr(args,'moveit_api','ros'),
                            tcp_profile=getattr(args,'tcp_profile','flange'),
                            payload_config=json.loads(args.payload) if getattr(args,'payload',None) else None,
                            native_smoothing=getattr(args,'native_smoothing','none'),
                            motion_profile=args.motion_profile,camera_session=getattr(args,'camera_session',None),
                            workspace_config=json.loads(args.test_workspace) if getattr(args,'test_workspace',None) else None)
    executor.memory_residency=getattr(args,'runtime_residency',{'mode':'off','locked_bytes':0})
    executor.start()
    from .operator_logs import start_logging,audit_message
    from .log_sources import LogSources
    log_root=(args.audit_dir.parent if args.audit_dir else Path('/tmp/e6-logs')/str(time.time_ns()))
    logger,log_sink=start_logging(log_root/'control.log','e6.control')
    logs=LogSources(session=getattr(args,'camera_session',None))
    logs.register('control',log_sink.path,root=log_root)
    if args.camera_recording_dir:logs.register('camera',args.camera_recording_dir/'human.log',root=log_root)
    if executor.moveit:
        def servo_log():
            # Only read the host-assigned immutable path reference. Log readers
            # must not acquire the solver snapshot/control locks.
            session=executor.moveit.info.get('session')
            return Path(session)/'servo.log' if session else None
        logs.register('servo',servo_log,root=executor.moveit.bundle/'sessions')
    logger.info('Gateway connected; stage=%s; TCP=%s; speed=%s; workspace=%s; joint range=%s',
        args.stage,executor.tool.description(),executor.motion_profile.description(),executor.workspace.description(),
        executor.joint_range.description() if executor.joint_range else None)
    logger.info('Configured payload (kg / default tool mm): %s; applied on explicit Enable; feedback checked before motion',
                executor.payload.description())
    audit=None
    if args.audit_dir:
        from .audit import Audit
        try:audit=Audit(args.audit_dir,executor.reader)
        except Exception:
            logger.exception('Audit initialization failed')
            logger.removeHandler(log_sink);log_sink.close()
            executor.close()
            raise
    episodes=None;media_server=None
    if args.episodes_dir:
        try:
            if args.camera_health is None:raise ValueError('Episode recording requires camera health directory')
            from .episodes import Episodes
            from .episode_http import start_server
            episodes=Episodes(args.episodes_dir,args.camera_recording_dir or args.camera_health.parent,executor.reader,args.stage,
                              camera_status_path=args.camera_health.with_name('recording.status.json'))
            executor.episodes=episodes
            media_server=start_server(episodes,args.media_port,logs=logs)
        except Exception:
            logger.exception('Recording/media initialization failed')
            logger.removeHandler(log_sink);log_sink.close()
            executor.close()
            if episodes:episodes.close()
            if audit:audit.close()
            lock.close()
            raise
    mutex=threading.RLock()
    halted=threading.Event()
    exit_event=asyncio.Event()
    loop=asyncio.get_running_loop()
    from .runtime_timing import RuntimeTiming
    # Observe gateway-local GC and scheduling separately from the ROS process.
    # A 20 Hz worker normally spends 50 ms between calls; 100 ms only filters
    # diagnostic history, never input/device deadlines or stop decisions.
    timing=RuntimeTiming(gap_slow_ms=100)
    timing.install()
    for sig in (signal.SIGINT,signal.SIGTERM):loop.add_signal_handler(sig,exit_event.set)

    def synchronized(method,*values):
        with mutex:return method(*values)

    def record(event):
        if audit:audit.put(event)
        if episodes:episodes.put(event)
        audit_message(logger,event)

    def record_feedback_events():
        for event in executor.reader.drain_events():
            record(dict(event='feedback_transport',at_ns=event['received_ns'],data=event))

    def tick():
        next_record_ns=0
        record_period_ns=round(executor.servo_period_s*1e9)
        while not halted.is_set():
            begin_ns=time.perf_counter_ns()
            timing.begin()
            try:
                record_feedback_events()
                timing.mark('record_feedback_events')
                synchronized(executor.tick)
                timing.mark('executor_tick_with_lock')
                grip_events=executor.gripper.drain() if executor.gripper else []
                control_events=synchronized(executor.drain_events)
                for event in control_events:record(event)
                for event in grip_events:record(event)
                timing.mark('record_control_events')
                if audit and audit.error:synchronized(executor.suspend,'证据写盘失败: '+audit.error)
                sample_ns=time.perf_counter_ns()
                if executor.moveit_api!='cpp' or sample_ns>=next_record_ns:
                    # Extra candidate wakeups service control, not duplicate
                    # observation frames. Every action event is still recorded.
                    record(dict(event='cycle',at_ns=sample_ns,state=capture_audit_observation(executor,mutex),
                                gateway_timing=timing.snapshot()))
                    next_record_ns=(sample_ns+record_period_ns if not next_record_ns else
                                    next_record_ns+((sample_ns-next_record_ns)//record_period_ns+1)*record_period_ns)
                timing.mark('state_and_audit')
                if episodes:synchronized(executor.check_recording)
            except Exception as exc:
                logger.exception('Control execution failed')
                synchronized(executor.suspend,'执行异常: '+str(exc))
            finally:timing.end()
            wake=executor.moveit.initial_candidate_event if executor.moveit else executor.native_wake
            last_sent=executor.last_moveit_sent_ns if executor.moveit else executor.last_native_sent_ns
            wait_control_cycle(halted,wake,begin_ns,executor.servo_period_s,last_sent,
                               producer_paced=executor.moveit is not None and executor.moveit_api=='cpp',
                               scheduled_ns=executor.scheduled_write_ns())

    control_slot=asyncio.Lock()

    async def handle_session(ws,hello):
        owner=id(ws)
        probes=ClockProbes()
        controlling=False
        try:
            controlling=hello.get('role')=='control'
            logger.info('%s connection opened: %s','Control' if controlling else 'Observation',owner)
            if controlling and not synchronized(executor.claim,owner):
                await ws.send(json.dumps(dict(kind='state',state='暂挂',ready=False,echo_ns=0,reason='已有控制者')))
                return
            async def receive():
                async for raw in ws:
                    received_ns=time.perf_counter_ns()
                    data=json.loads(raw)
                    if data.get('op')=='clock_probe':
                        probe=probes.issue(data.get('client_sent_ms'),received_ns,time.perf_counter_ns())
                        if probe:await ws.send(json.dumps(probe))
                        continue  # Clock measurements never renew an intent or lease.
                    if data.get('op')=='clock_sample':
                        measurement=probes.complete(data,received_ns)
                        if measurement:
                            if controlling:
                                def observe_clock():
                                    if executor.owner==owner and executor.session:
                                        executor.session.observe_clock(measurement)
                                await asyncio.to_thread(synchronized,observe_clock)
                            record(dict(event='clock_sample',at_ns=received_ns,
                                connection_id=owner,data=measurement))
                        continue  # Invalid diagnostics do not suspend an otherwise healthy controller.
                    if controlling:
                        executor.notify_received(owner,data)
                        record(dict(event='input',at_ns=received_ns,connection_id=owner,data=data))
                        def apply_input():
                            processing_ns=time.perf_counter_ns()
                            executor.receive(data)
                            session=executor.session
                            return dict(event='input_processed',at_ns=time.perf_counter_ns(),
                                connection_id=owner,
                                received_ns=received_ns,processing_ns=processing_ns,sequence=data.get('seq'),
                                last_accepted_input_ns=session.received_ns if session else None,
                                ready=bool(session and session.ready),reason=session.reason if session else None,
                                dropped=session.dropped if session else None,
                                freshness=session.last_freshness if session else None)
                        record(await asyncio.to_thread(synchronized,apply_input))
            async def publish():
                while True:
                    state=await asyncio.to_thread(observe_executor,executor,mutex,controlling)
                    if controlling:state=for_operator(state)
                    await asyncio.wait_for(ws.send(json.dumps(state)),.1)
                    await asyncio.sleep(.05 if controlling else .2)
            tasks=[asyncio.create_task(receive()),asyncio.create_task(publish())]
            try:
                done,_=await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
                for task in done:task.result()
            finally:
                for task in tasks:task.cancel()
                await asyncio.gather(*tasks,return_exceptions=True)
        except (ConnectionClosed,asyncio.TimeoutError,ValueError):
            pass
        finally:
            logger.info('Connection closed: %s',owner)
            if controlling:
                # A rejected second tab does not own the first tab's episode.
                executor.notify_disconnected(owner)
                if executor.owner==owner and episodes:episodes.abort('控制连接关闭')
                cleanup=asyncio.create_task(asyncio.to_thread(synchronized,executor.release,owner))
                try:
                    await asyncio.shield(cleanup)
                except asyncio.CancelledError:
                    # The admission slot must outlive ownership cleanup, even
                    # if the socket handler itself is cancelled during exit.
                    await cleanup
                    raise

    async def handle(ws):
        try:
            hello=json.loads(await asyncio.wait_for(ws.recv(),3))
            if hello.get('role')!='control':
                await handle_session(ws,hello)
                return
            async with controller_admission(control_slot) as admitted:
                if not admitted:
                    record(dict(event='control_refused',at_ns=time.perf_counter_ns(),reason='已有控制者'))
                    await ws.send(json.dumps(dict(kind='state',state='暂挂',ready=False,echo_ns=0,reason='已有控制者')))
                    return
                await handle_session(ws,hello)
        except (ConnectionClosed,asyncio.TimeoutError,ValueError):
            pass

    task=threading.Thread(target=tick,name='e6-local-control',daemon=True)
    task.start()
    try:
        async with serve(handle,'127.0.0.1',args.port,max_size=16384,max_queue=4,
                         compression=None,ping_interval=5,ping_timeout=5):
            await exit_event.wait()
    finally:
        halted.set()
        await asyncio.to_thread(task.join,2)
        try:await asyncio.to_thread(synchronized,executor.close)
        finally:
            try:
                if audit or episodes:
                    record_feedback_events()
                    # The CAN worker may finish one in-flight exchange after
                    # the control tick exits. Preserve it before the writer
                    # drains; otherwise interface counters exceed the log.
                    if executor.gripper:
                        for event in executor.gripper.drain():record(event)
                    for event in executor.drain_events():record(event)
                    record(dict(event='shutdown_state',at_ns=time.perf_counter_ns(),state=executor.state(),
                                gateway_timing=timing.snapshot()))
                    try:
                        if episodes:await asyncio.to_thread(episodes.close)
                    finally:
                        if audit:await asyncio.to_thread(audit.close)
            finally:
                timing.close()
                if media_server:
                    await asyncio.to_thread(media_server.shutdown)
                    media_server.server_close()
                lock.close()
                logger.info('Gateway shutdown completed')
                logger.removeHandler(log_sink);log_sink.close()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--host',default='192.168.1.4')
    p.add_argument('--port',type=int,default=8776)
    p.add_argument('--stage',choices=['shadow','live'],default='shadow')
    p.add_argument('--camera-health',type=Path)
    p.add_argument('--camera-session',help='Session resource owner for explicit camera restart')
    p.add_argument('--camera-recording-dir',type=Path,help='Disk camera archive directory when health uses a separate tmpfs path')
    p.add_argument('--audit-dir',type=Path)
    p.add_argument('--episodes-dir',type=Path)
    p.add_argument('--media-port',type=int,default=8777)
    p.add_argument('--acceptance-inputs',action='store_true')
    p.add_argument('--memory-residency',choices=['off','current'],default='off',
                   help='Linux startup-only current-runtime residency qualification')
    p.add_argument('--servo-period-s',type=float,choices=[.03,.05],default=.05,
                   help='Native ServoP/ServoJ period; 30 ms follows official cadence recommendation')
    from .session_runner import DEFAULT_SERVO_DURATION_S
    p.add_argument('--servo-duration-s',type=float,choices=[.05,.1],default=DEFAULT_SERVO_DURATION_S,
                   help='SDK point duration t; default/live 0.1 s, independent of the 20 Hz sender period')
    p.add_argument('--servo-aheadtime',type=int,choices=[20,50],default=50)
    p.add_argument('--servo-gain',type=int,choices=[500,750],default=500,
                   help='Per-command native gain qualification; not a persistent controller parameter')
    p.add_argument('--release-mode',choices=['immediate','ruckig'],default='immediate',
                   help='Ruckig is an explicit release-deceleration qualification; faults still use Stop')
    p.add_argument('--moveit-bundle',type=Path,help='Explicit qualified MoveIt Servo/model deployment; default remains native')
    p.add_argument('--moveit-api',choices=['ros','cpp'],default='ros')
    p.add_argument('--motion-profile',choices=['commissioning','intermediate','normal'],default='commissioning')
    p.add_argument('--native-smoothing',choices=['none','ruckig'],default='none',
                   help='Native active-input smoothing only; normal release still immediately sends Stop')
    p.add_argument('--tcp-profile',choices=['flange','omnipicker-r2','omnipicker-r4'],default='flange')
    p.add_argument('--payload',help='Session-fixed JSON: mass_kg, center_mm; applied on explicit Enable')
    p.add_argument('--test-workspace',help='Session-fixed JSON: position_m, rotation_deg, joint_deg; defaults 0.05/10/10')
    args=p.parse_args()
    # These modules previously entered the old generation on the live path.
    # Import and collect before opening devices, then use CPython's built-in
    # permanent generation for this immutable process-lifetime graph. Fresh
    # connections, records and input objects remain normally collectible.
    from . import audit, episodes, episode_http, gripper, jazzy_host, latest_json_files, moveit_bridge
    from .runtime_timing import frozen_startup_heap, cooperative_thread_scheduling
    from .memory_residency import resident_current
    with frozen_startup_heap() as memory_policy, cooperative_thread_scheduling() as thread_policy, resident_current(args.memory_residency) as residency:
        print(json.dumps(dict(event='runtime_memory_policy', **memory_policy)),flush=True)
        print(json.dumps(dict(event='runtime_thread_policy', **thread_policy)),flush=True)
        print(json.dumps(dict(event='runtime_residency', **residency)),flush=True)
        args.runtime_residency=residency
        asyncio.run(run(args))


if __name__=='__main__':main()
