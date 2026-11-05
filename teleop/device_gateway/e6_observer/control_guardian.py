"""Independent X5 process: owns Dashboard, passive feedback, CAN and watchdog.

There is no general command passthrough. Only this process owns port 29999.
"""
import base64
from collections import deque
import json
import os
import queue
import re
import socket
import threading
import time
from .control_core import ControlCore,ControlLimits,InputReleased,ControlCancelled
from .protocol import FeedbackParser
from .feedback_diagnostics import FeedbackTrace


def recent_command_events(events):
    """Preserve the public 20-event window before multiprocessing pickles it.

    The full Dashboard audit ring remains available in this process. Sending
    its older entries to the camera process only to discard them there adds
    serialization work as the live session accumulates commands.
    """
    return list(events)[-20:]


def configure_e6_socket_priority(sock):
    """Request UP7 for this owned E6 socket, without changing global QoS."""
    sock.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,224)
    profile=dict(requested_ip_tos=224,effective_ip_tos=sock.getsockopt(socket.IPPROTO_IP,socket.IP_TOS),
                 requested_socket_priority=None,effective_socket_priority=None,wire_80211_priority_verified=False)
    if profile['effective_ip_tos']!=224:raise ValueError('Owned E6 socket IP priority did not apply')
    if hasattr(socket,'SO_PRIORITY'):
        # cfg80211 accepts priorities 256..263 as explicit user priorities
        # 0..7. Installed AIC select_queue calls that classifier. This readback
        # verifies the socket request, not AP scheduling or over-air delivery.
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_PRIORITY,263)
        value=sock.getsockopt(socket.SOL_SOCKET,socket.SO_PRIORITY)
        if value!=263:raise ValueError('Owned E6 socket priority did not apply')
        profile.update(requested_socket_priority=263,effective_socket_priority=value)
    return profile


class DashboardLink:
    def __init__(self,ip):
        self.ip=ip;self.sock=None;self.events=deque(maxlen=256);self.motion_tx=0;self.stop_tx=0
        self.gripper=None;self.cancel=threading.Event();self.io_lock=threading.RLock();self._connect()
        self.release_pending=False
        self.cancel_fault_reason=None
        self.late_motion_reply=None
        self.stop_state=None;self.stop_thread=None;self.stop_state_lock=threading.Lock()
    def _connect(self,timeout=2):
        self.sock=socket.create_connection((self.ip,29999),timeout=timeout)
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.socket_priority=configure_e6_socket_priority(self.sock)
            if hasattr(socket,'TCP_QUICKACK'):self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_QUICKACK,1)
        except BaseException:
            self.sock.close();self.sock=None;raise
    def query(self,cmd,deadline=None):
        if getattr(self,'late_motion_reply',None):
            raise ValueError('Unconfirmed motion; only independent Stop is allowed')
        if getattr(self,'stop_state',None) and not self.stop_state.get('confirmation_complete'):
            raise ValueError('Stop confirmation pending; Dashboard writes disabled')
        with self.io_lock:
            if getattr(self,'stop_state',None) and not self.stop_state.get('confirmation_complete'):
                raise ValueError('Stop confirmation pending; Dashboard writes disabled')
            return self._query(cmd,deadline)
    def reconnect_for_arm(self):
        # A discarded late-reply connection is replaced only on an explicit
        # neutral preparation request. This method sends no device command.
        with self.io_lock:
            if self.sock is None:self._connect(.05)
    def _query(self,cmd,deadline=None):
        allowed=cmd in ('RobotMode()','GetAngle()','GetPose()','GetErrorID()','EnableRobot()','Stop()') or cmd.startswith(('InverseKin(','PositiveKin(','ServoJ('))
        if not allowed:raise ValueError('Guardian command whitelist')
        if cmd.startswith('ServoJ(') and self.cancel.is_set():
            if self.release_pending:raise InputReleased('允许动作键已释放，ServoJ未发送')
            raise ControlCancelled(getattr(self,'cancel_fault_reason',None) or 'Stop pending; suppress ServoJ')
        deadline=deadline or time.perf_counter_ns()+50_000_000
        remaining=(deadline-time.perf_counter_ns())/1e9
        if remaining<=0:raise TimeoutError('Command deadline expired before send')
        entry=dict(command=cmd,x5_sent_ns=time.perf_counter_ns(),acknowledged=False,send_attempted=False,send_confirmed=False,acceptance='not_sent')
        entry['socket_priority']=dict(getattr(self,'socket_priority',{}))
        self.events.append(entry)
        if hasattr(socket,'TCP_INFO') and hasattr(self.sock,'getsockopt'):
            try:
                from .ssh_socket_qos import tcp_timing
                entry['tcp_timing']=dict(tcp_timing(self.sock),measured_on='X5 to E6 Dashboard socket')
            except OSError as exc:entry['tcp_timing_unavailable']=str(exc)
        if cmd.startswith('ServoJ('):self.motion_tx+=1
        if cmd=='Stop()':self.stop_tx+=1
        data=bytearray()
        try:
            self.sock.settimeout(remaining);entry['send_attempted']=True;self.sock.sendall(cmd.encode('ascii'))
            entry.update(send_confirmed=True,send_completed_ns=time.perf_counter_ns(),acceptance='unconfirmed')
            while b';' not in data:
                if cmd!='Stop()' and self.cancel.is_set() and not self.release_pending:raise ControlCancelled(getattr(self,'cancel_fault_reason',None) or 'Control transaction cancelled')
                remaining=(deadline-time.perf_counter_ns())/1e9
                if remaining<=0:raise TimeoutError('Command reply deadline')
                self.sock.settimeout(min(.02,remaining))
                try:chunk=self.sock.recv(4096)
                except socket.timeout:continue
                if not chunk:raise ConnectionError('Dashboard closed: '+data.decode('utf-8','replace')[:200])
                data.extend(chunk)
                if hasattr(socket,'TCP_QUICKACK'):self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_QUICKACK,1)
                if len(data)>65536:raise ValueError('Dashboard oversized reply')
            raw=data.decode('utf-8').strip();entry['raw_reply']=raw;entry['x5_reply_ns']=time.perf_counter_ns()
            match=re.fullmatch(r'\s*(-?\d+)\s*,\s*\{(.*?)\}\s*,\s*(.*?)\s*;',raw,re.S)
            if not match or re.sub(r'\s+','',match[3])!=re.sub(r'\s+','',cmd):raise ValueError('Dashboard mismatched response')
            if int(match[1])!=0:
                entry['acceptance']='controller_rejected';raise ValueError('Dashboard rejected command: '+raw)
            entry.update(acknowledged=True,acceptance='controller_acknowledged');return match[2].strip()
        except Exception as exc:
            entry['error']=str(exc)
            entry['uncertain']=bool(cmd.startswith('ServoJ(') and entry['send_attempted'] and not entry['acknowledged'] and entry['acceptance']!='controller_rejected')
            # Do not open a second controller connection while a motion may
            # still be executing. Keep this socket exclusively for Stop and
            # correlate its preceding late reply by the exact command echo.
            # The timed-out action remains uncertain, even if its reply later
            # arrives. No further query, target or automatic resend is allowed.
            if entry['uncertain'] and isinstance(exc,(TimeoutError,InterruptedError)):
                self.late_motion_reply=dict(command=cmd,buffer=bytes(data),event=entry)
            else:self.close_socket()
            raise
    def ik(self,p,q,deadline):
        cmd='InverseKin('+','.join(f'{x:.6f}' for x in p)+',user=0,tool=0,useJointNear=1,jointNear={'+','.join(f'{x:.6f}' for x in q)+'})'
        return [float(x) for x in self.query(cmd,deadline).split(',')]
    def fk(self,q,deadline):return [float(x) for x in self.query('PositiveKin('+','.join(f'{x:.6f}' for x in q)+',user=0,tool=0)',deadline).split(',')]
    def servo(self,q,deadline):
        result=self.query('ServoJ('+','.join(f'{x:.6f}' for x in q)+',t=0.05,aheadtime=50,gain=500)',deadline)
        if not re.fullmatch(r'\d+',result):raise ValueError('Invalid ServoJ command ID')
        return int(result)
    def stop(self):
        with self.io_lock:
            deadline=time.perf_counter_ns()+50_000_000
            if self.sock is None:self._connect(.05)
            return self.query('Stop()',deadline)
    def begin_stop(self):
        """Send once within 50 ms; collect only its reply on the same socket.

        Stop is not a motion transaction. Its separate 1 s confirmation
        deadline never permits another target, query, resend or rearm.
        """
        self.cancel.set()
        with self.stop_state_lock:
            if self.stop_state and not self.stop_state['confirmation_complete']:
                return dict(self.stop_state)
        requested=time.perf_counter_ns();deadline=requested+50_000_000
        with self.io_lock:
            if self.sock is None:self._connect(max(.001,(deadline-time.perf_counter_ns())/1e9))
            entry=dict(command='Stop()',x5_sent_ns=time.perf_counter_ns(),stop_requested_ns=requested,
                acknowledged=False,send_attempted=False,send_confirmed=False,acceptance='not_sent',
                confirmation_complete=False,send_budget_ms=50,confirmation_budget_ms=1000)
            entry['socket_priority']=dict(getattr(self,'socket_priority',{}))
            with self.stop_state_lock:self.stop_state=entry
            self.events.append(entry);self.stop_tx+=1
            try:
                remaining=(deadline-time.perf_counter_ns())/1e9
                if remaining<=0:raise TimeoutError('Stop send deadline')
                self.sock.settimeout(remaining);entry['send_attempted']=True;self.sock.sendall(b'Stop()')
                entry.update(send_confirmed=True,send_completed_ns=time.perf_counter_ns(),acceptance='unconfirmed')
                entry['send_duration_ms']=(entry['send_completed_ns']-requested)/1e6
                if entry['send_completed_ns']>deadline:raise TimeoutError('Stop send exceeded 50ms')
            except Exception as exc:
                entry.update(error=str(exc),confirmation_complete=True);self.close_socket();raise
        self.stop_thread=threading.Thread(target=self._confirm_stop,args=(entry,),daemon=True)
        self.stop_thread.start()
        return self.poll_stop()
    def _confirm_stop(self,entry):
        with self.io_lock:
            try:
                late=getattr(self,'late_motion_reply',None)
                deadline=entry['send_completed_ns']+1_000_000_000;data=bytearray(late['buffer'] if late else b'')
                while True:
                    if b';' in data:
                        raw_bytes,_,tail=data.partition(b';');data=bytearray(tail)
                        raw=(raw_bytes+b';').decode('utf-8').strip()
                        match=re.fullmatch(r'\s*(-?\d+)\s*,\s*\{(.*?)\}\s*,\s*(.*?)\s*;',raw,re.S)
                        if not match:raise ValueError('Dashboard malformed response during Stop')
                        echo=re.sub(r'\s+','',match[3])
                        if late:
                            if echo!=re.sub(r'\s+','',late['command']):raise ValueError('Dashboard mismatched late motion response')
                            late['event'].update(late_raw_reply=raw,late_reply_receive_ns=time.perf_counter_ns(),late_reply_ignored_for_action=True)
                            entry['preceding_uncertain_motion_reply_drained']=True
                            self.late_motion_reply=None;late=None
                            continue
                        if echo!='Stop()':raise ValueError('Dashboard mismatched Stop response')
                        if data.strip():raise ValueError('Dashboard extra response after Stop')
                        entry.update(raw_reply=raw,x5_reply_ns=time.perf_counter_ns())
                        if int(match[1])!=0:raise ValueError('Dashboard rejected Stop: '+raw)
                        entry.update(acknowledged=True,acceptance='controller_acknowledged',
                            confirmation_duration_ms=(entry['x5_reply_ns']-entry['send_completed_ns'])/1e6)
                        break
                    remaining=(deadline-time.perf_counter_ns())/1e9
                    if remaining<=0:raise TimeoutError('Stop confirmation deadline (1000ms)')
                    self.sock.settimeout(min(.02,remaining))
                    try:chunk=self.sock.recv(4096)
                    except socket.timeout:continue
                    if not chunk:raise ConnectionError('Dashboard closed during Stop confirmation')
                    data.extend(chunk)
                    if len(data)>65536:raise ValueError('Dashboard oversized Stop response')
            except Exception as exc:
                entry['error']=str(exc);self.close_socket()
            finally:
                with self.stop_state_lock:entry['confirmation_complete']=True
    def poll_stop(self):
        with self.stop_state_lock:return None if self.stop_state is None else dict(self.stop_state)
    def grip(self,target):
        if self.gripper is None:raise ValueError('CAN not independently initialized')
        if self.cancel.is_set():
            if self.release_pending:raise InputReleased('允许动作键已释放，CAN未发送')
            raise ControlCancelled(getattr(self,'cancel_fault_reason',None) or 'CAN command cancelled')
        return self.gripper.exchange(target)
    def close_socket(self):
        if self.sock:
            try:self.sock.close()
            except OSError:pass
        self.sock=None
        self.late_motion_reply=None
    def close(self):
        if getattr(self,'stop_thread',None):self.stop_thread.join(1.1)
        if self.gripper:self.gripper.close()
        self.close_socket()

class LocalDashboardLink(DashboardLink):
    """Local cycle computation; native preparation and per-feedback auditing.

    The unique Dashboard socket still owns ServoJ, Stop and commissioning.
    No background native query competes with an active control transaction.
    """
    def __init__(self,ip,profile):
        import numpy as np
        from .kinematics_profile import load_profile
        self.model,self.kinematics_status=load_profile(profile,ip)
        self.kinematics_status.update(prepared=False,feedback_checks=0)
        self.controller_time=None;self.controller_runtime=None
        self.native_preparation_reference=None
        self.reconnect_ready_after_ns=0
        self.connection_staging_thread=None;self.connection_staging_error=None;self.connection_staging_closed=False
        # Warm NumPy/BLAS in the independent guardian before it becomes ready.
        p,r=self.model.fk(self.model.anchor_q)
        for _ in range(3):self.model.solve(p+[.000001,0,0],r,self.model.anchor_q)
        super().__init__(ip)
        self.connection_started_ns=time.perf_counter_ns()
    def connection_ready(self):
        age=time.perf_counter_ns()-self.connection_started_ns
        staging=getattr(self,'connection_staging_thread',None)
        self.kinematics_status.update(connection_age_ms=age/1e6,
            connection_staging=bool(staging and staging.is_alive()),
            connection_staging_error=getattr(self,'connection_staging_error',None),
            connection_ready=bool(self.sock and age>=1_000_000_000 and not (staging and staging.is_alive())
                and not getattr(self,'connection_staging_error',None) and not getattr(self,'connection_staging_closed',False)))
        return self.kinematics_status['connection_ready']
    def stage_neutral_connection(self):
        # Resource acquisition during valid neutral, disarmed input. No native
        # query, enable, Stop, ServoJ or CAN; it cannot clear a control fault.
        # Let the controller accept the socket before explicit preparation.
        now=time.perf_counter_ns()
        if self.stop_state and not self.stop_state['confirmation_complete']:return
        if getattr(self,'connection_staging_closed',False):raise ValueError('Connection staging closed')
        if getattr(self,'connection_staging_error',None):raise ValueError(self.connection_staging_error)
        staging=getattr(self,'connection_staging_thread',None)
        if staging and staging.is_alive():return
        # A retained startup connection was reset by the controller on its
        # first query after ~23 s of silence. Retire only while disarmed,
        # neutral and before preparation; never retry a failed transaction.
        if self.sock is not None and now-self.connection_started_ns>10_000_000_000:
            self.close_socket();return
        if self.sock is None and now>=self.reconnect_ready_after_ns:
            # TCP acquisition sends no E6 command. Do it outside the guardian
            # control lock: a slow SYN must not stall fresh neutral input or
            # the 20 Hz ticker. Never retry a failed acquisition in-session.
            def acquire():
                try:
                    with self.io_lock:
                        self._connect(1)
                        self.connection_started_ns=time.perf_counter_ns()
                        if getattr(self,'connection_staging_closed',False):self.close_socket()
                except Exception as exc:
                    self.close_socket()
                    self.connection_staging_error='Neutral connection staging failed: '+str(exc)
            self.connection_staging_thread=threading.Thread(target=acquire,daemon=True)
            self.connection_staging_thread.start()
    def close(self):
        self.connection_staging_closed=True
        staging=getattr(self,'connection_staging_thread',None)
        if staging:staging.join(1.1)
        super().close()
    def close_socket(self):
        existed=self.sock is not None
        super().close_socket()
        if existed:self.reconnect_ready_after_ns=time.perf_counter_ns()+2_000_000_000
    def validate_feedback(self,d,*,streamed=False):
        import math
        import numpy as np
        from .control_math import pose,angle
        q=np.asarray(d['joints_rad']);p,r=self.model.fk(q)
        actual_p,actual_r=pose(d['controller_pose_raw_mm_deg'])
        position=float(np.linalg.norm(p-actual_p));rotation=angle(r,actual_r)
        self.kinematics_status.update(actual_residual_m=position,actual_residual_rad=rotation,
                                      feedback_checks=self.kinematics_status['feedback_checks']+1)
        if position>.001 or rotation>math.radians(1):raise ValueError('局部模型与控制器实际位姿偏差超过1mm/1°')
        for field,attribute in (('controller_timestamp_raw','controller_time'),('controller_runtime_raw','controller_runtime')):
            value=d.get(field);previous=getattr(self,attribute)
            if type(value) is not int or streamed and previous is not None and value<previous:raise ValueError('控制器时钟重置，局部模型会话失效')
            if streamed:setattr(self,attribute,value)
    def prepare(self,d,deadline):
        import math
        import numpy as np
        from .control_math import pose,angle
        self.kinematics_status['prepared']=False
        if not self.connection_ready():
            raise ValueError('连接预备尚未就绪，保持回中后显式准备')
        self.cancel.clear();self.release_pending=False
        self.validate_feedback(d)
        begin=time.perf_counter_ns();q=d['q_actual_deg'];raw=d['controller_pose_raw_mm_deg']
        p,r=pose(raw)
        reference=self.native_preparation_reference
        # An input-owner change does not invalidate successful read-only
        # calibration at the same stationary configuration. Keep its original
        # receive time and explicitly expose reuse. Every current feedback is
        # still validated; restart, domain/pose drift and configuration changes
        # remain faults, and a different posture needs fresh native queries.
        if reference:
            rp,rr=pose(reference['pose_mm_deg'])
            same=(np.max(np.abs(np.asarray(q)-reference['joints_deg']))<=.01
                and np.linalg.norm(p-rp)<=.0001 and angle(r,rr)<=math.radians(.01)
                and d['controller_timestamp_raw']>=reference['controller_timestamp_raw']
                and d['controller_runtime_raw']>=reference['controller_runtime_raw'])
            if same:
                self.kinematics_status.update(prepared=True,native_preparation_reused=True,
                    native_preparation_proof_age_ms=(begin-reference['verified_ns'])/1e6)
                return
        native_q=DashboardLink.ik(self,raw,q,deadline)
        native=DashboardLink.fk(self,native_q,deadline)
        fp,fr=pose(native)
        if (len(native_q)!=6 or not np.isfinite(native_q).all() or np.max(np.abs(np.asarray(native_q)-q))>.05
                or np.linalg.norm(p-fp)>.001 or angle(r,fr)>math.radians(1)):
            raise ValueError('准备阶段原生IK/FK复核失败')
        self.model.check_domain(np.radians(native_q))
        self.native_preparation_reference=dict(joints_deg=list(q),pose_mm_deg=list(raw),verified_ns=time.perf_counter_ns(),
            controller_timestamp_raw=d['controller_timestamp_raw'],controller_runtime_raw=d['controller_runtime_raw'])
        self.kinematics_status.update(prepared=True,native_preparation_reused=False,native_preparation_proof_age_ms=0,
                                     native_preparation_ms=(time.perf_counter_ns()-begin)/1e6,
                                     native_preparation_joints_deg=native_q,native_preparation_pose_mm_deg=native)
    def ik(self,raw,seed,deadline):
        import numpy as np
        from .control_math import pose
        if not self.kinematics_status['prepared']:raise ValueError('局部模型未完成原生准备复核')
        if time.perf_counter_ns()>deadline:raise TimeoutError('Local IK deadline')
        begin=time.perf_counter_ns();cpu=time.thread_time_ns();residual=None
        try:
            q,residual=self.model.solve(*pose(raw),np.radians(seed))
            return np.degrees(q).tolist()
        finally:
            self.kinematics_status['last_local_ik_timing']=dict(wall_ms=(time.perf_counter_ns()-begin)/1e6,
                thread_cpu_ms=(time.thread_time_ns()-cpu)/1e6,iterations=residual and residual['iterations'])
    def fk(self,q,deadline):
        import numpy as np
        from .control_math import controller_pose
        if time.perf_counter_ns()>deadline:raise TimeoutError('Local FK deadline')
        p,r=self.model.fk(np.radians(q))
        return controller_pose(p,r,[0]*6)

class GripperLink:
    def __init__(self,profile='plan32'):
        from .gripper_profile import gripper_profile
        settings=gripper_profile(profile)
        if settings.get('enabled',True) is False:
            raise ValueError('夹爪未通过验收，本次会话禁止配置CAN及发送')
        import fcntl
        from pathlib import Path
        from omnipicker.tools import verify_can_control as can
        self.command_parameters={k:settings[k] for k in ('velocity','force','acceleration','deceleration')}
        self.can=can;self.channel=None;self.lock=None;self.configured=False;self.feedback=None
        self.report=dict(events=[]);self.last_target=None;self.last_command=None;self.tx_count=0;self.request_count=0
        self.minimum_target_delta_counts=settings.get('minimum_target_delta_counts',1)
        try:
            self.lock=open('/tmp/omnipicker-can1.lock','a');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            can.validate_usb_interface('can1');before=can.ip_snapshot('can1')
            if 'UP' in before.get('flags',[]):raise ValueError('CAN already UP; refuse takeover')
            can.configure('can1','type','can','bitrate','1000000','sample-point','0.8','listen-only','off')
            can.configure('can1','up');self.configured=True
            self.channel=can.CanChannel('can1',1,.025,self.report)
        except BaseException:self.close();raise
    def exchange(self,target):
        from .gripper_profile import select_command
        requested=self.can.encode_command(target*100,self.command_parameters,bounds=(15,60))
        previous=getattr(self,'last_command',None)
        command=select_command(requested,previous,getattr(self,'minimum_target_delta_counts',1))
        self.request_count+=1;begin=len(self.report['events'])
        try:f=self.channel.exchange(command)
        finally:self.tx_count+=sum(e.get('kind')=='tx' and e.get('send_confirmed',False) for e in self.report['events'][begin:])
        self.last_command=command;self.last_target=command[1]/255
        self.feedback=dict(f,opening=f['position_raw']/255,x5_receive_monotonic_ns=f['host_perf_counter_ns'],
                           source='can1 request-associated feedback, not cryptographic authentication',seq=self.request_count,valid=True,available=True,
                           requested_opening=target,command_position_raw=command[1],command_opening=command[1]/255,
                           command_target_changed=previous is None or command[1]!=previous[1],
                           minimum_target_delta_counts=getattr(self,'minimum_target_delta_counts',1))
        # Keep diagnostics bounded during a long session.
        self.report['events']=self.report['events'][-256:]
        return self.feedback
    def close(self):
        if self.channel:self.channel.close();self.channel=None
        if self.configured:
            self.can.configure('can1','down');self.configured=False
        if self.lock:self.lock.close();self.lock=None

def put_latest(q,value):
    try:q.put_nowait(value)
    except queue.Full:
        try:q.get_nowait()
        except queue.Empty:pass
        try:q.put_nowait(value)
        except queue.Full:pass


class Commissioner:
    """Long initialization tasks never hold the tick/status lock while doing IO."""
    def __init__(self,core,link,lock,fresh,parent_alive,gripper_factory=GripperLink):
        self.core,self.link,self.lock,self.fresh,self.parent_alive=core,link,lock,fresh,parent_alive
        self.gripper_factory=gripper_factory;self.thread=None;self.gripper_attempted=False
    def start(self,operation):
        c=self.core
        diagnostic=operation=='diagnose-kinematics'
        if (c.stage!='shadow' if diagnostic else c.stage!='live') or c.armed or self.thread and self.thread.is_alive():raise ValueError('Independent initialization/diagnosis rejected')
        s=self.fresh();now=time.perf_counter_ns()
        if not s or now-s['x5_receive_monotonic_ns']>100_000_000 or s['decoded']['error_status'] or s['decoded'].get('collision_state'):raise ValueError('Initialization requires fresh normal feedback')
        if diagnostic:pass
        elif operation=='enable':
            if c.enabled_once or s['decoded']['robot_mode']!=4:raise ValueError('Enable requires disabled idle and may be attempted once')
            c.enabled_once=True
        elif operation=='initialize-gripper':
            if (getattr(c,'gripper_command_profile',None) or {}).get('enabled',True) is False:
                raise ValueError('夹爪未通过验收，本次会话禁止初始化')
            if self.gripper_attempted:raise ValueError('Gripper initialization may be attempted once')
            if s['decoded']['robot_mode']!=5 or s['decoded']['enable_status']!=1:raise ValueError('Gripper initialization requires enabled idle')
            self.gripper_attempted=True
        else:raise ValueError('Unknown initialization task')
        c.commissioning=dict(operation=operation,state='running',started_ns=now)
        self.link.cancel.clear()
        self.thread=threading.Thread(target=self.run,args=(operation,),daemon=True);self.thread.start()
        return c.status()
    def check(self):
        if not self.parent_alive() or self.link.cancel.is_set():raise InterruptedError('Initialization cancelled or parent lost')
    def run(self,operation):
        c=self.core
        try:
            self.check()
            if operation=='diagnose-kinematics':
                samples=[]
                for _ in range(20):
                    self.check();s=self.fresh();d=s['decoded'];q=d['q_actual_deg'];p=d['controller_pose_raw_mm_deg']
                    if self.link.sock is None:self.link._connect()
                    begin=time.perf_counter_ns();ik=DashboardLink.ik(self.link,p,q,begin+1_000_000_000);middle=time.perf_counter_ns()
                    fk=DashboardLink.fk(self.link,ik,middle+1_000_000_000);end=time.perf_counter_ns()
                    samples.append(dict(ik_ms=(middle-begin)/1e6,fk_ms=(end-middle)/1e6,pair_ms=(end-begin)/1e6,ik_joints_deg=ik,fk_pose_mm_deg=fk,actual_joints_deg=q,actual_pose_mm_deg=p,x5_feedback_receive_ns=s['x5_receive_monotonic_ns'],diagnostic_only=True,control_budget_passed=end-begin<=50_000_000))
                    with self.lock:c.commissioning['samples']=list(samples)
                    self.link.cancel.wait(.05)
            elif operation=='enable':self.link.query('EnableRobot()',time.perf_counter_ns()+15_000_000_000)
            else:
                self.link.gripper=self.gripper_factory();stable=0;deadline=time.monotonic()+15
                while time.monotonic()<deadline:
                    self.check();f=self.link.grip(.15)
                    stable=stable+1 if f['state']==0 and abs(f['position_raw']-round(.15*255))<=1 else 0
                    if stable>=3:break
                    self.link.cancel.wait(.05)
                if stable<3:raise ValueError('Gripper 15% initialization timeout')
                with self.lock:c.grip_target=.15;c.gripper_initialized=True
            self.check()
            with self.lock:c.commissioning.update(state='complete',finished_ns=time.perf_counter_ns())
        except Exception as exc:
            self.link.cancel.set()
            with self.lock:
                if operation=='initialize-gripper':c.gripper_faulted=True
                c.commissioning.update(state='failed',error=str(exc),finished_ns=time.perf_counter_ns())
                c.stop('初始化失败: '+str(exc),force=operation=='enable',fault=True)

def run_guardian(ip,stage,role,token,limits,metadata_queue,result_queue,parent_heartbeat,shutdown,ready,link_factory=DashboardLink,feedback_source=None,listen_port=8768,kinematics_profile=None,feedback_framing='strict',recovery_policy='manual',gripper_command_profile='plan32'):
    """Spawn entry point. Queue payloads contain metadata only, never video pixels."""
    from websockets.sync.server import serve
    link=None;core=None;feedback=None;media={};fatal=None;tick_result=None;feedback_count=0;feedback_first_ns=None;feedback_failure=None;length_envelopes=0;feedback_socket_options=None;watchdog_failure=None
    lock=threading.RLock();feedback_lock=threading.Lock();done=threading.Event();commissioner=None
    try:
        # Tiny NumPy kernels release the GIL repeatedly. The passive reader
        # must not take a default 5 ms Python slice between every small kernel.
        # Limit this to the dedicated child interpreter, never the PC/camera
        # or in-process test interpreter. All wall-clock deadlines stay intact.
        import multiprocessing,sys
        dedicated=multiprocessing.current_process().name!='MainProcess'
        if dedicated:sys.setswitchinterval(min(sys.getswitchinterval(),.001))
        runtime_profile=dict(dedicated_process=dedicated,python_switch_interval_s=sys.getswitchinterval())
        if dedicated:
            # On the fixed 8-core X5, USB/SDIO IRQ work is concentrated on
            # the first CPUs. Constrain only this owned process and inherited
            # threads to the last two eligible cores; leave camera/system
            # affinity and every IRQ configuration untouched. Normal CFS,
            # not a realtime scheduling policy.
            original_cpus=sorted(os.sched_getaffinity(0));original_nice=os.getpriority(os.PRIO_PROCESS,0)
            selected=original_cpus[-2:] if len(original_cpus)>=4 else original_cpus
            os.sched_setaffinity(0,set(selected))
            os.setpriority(os.PRIO_PROCESS,0,min(original_nice,-5))
            runtime_profile.update(original_cpu_affinity=original_cpus,cpu_affinity=sorted(os.sched_getaffinity(0)),
                                   original_nice=original_nice,nice=os.getpriority(os.PRIO_PROCESS,0),scheduling_policy=os.sched_getscheduler(0))
        if kinematics_profile and link_factory is not DashboardLink:raise ValueError('Local calibration cannot be combined with a substitute device')
        link=LocalDashboardLink(ip,kinematics_profile) if kinematics_profile else link_factory(ip)
        core=ControlCore(link,stage,ControlLimits(**limits),recovery_policy=recovery_policy)
        from .gripper_profile import gripper_profile
        settings=gripper_profile(gripper_command_profile)
        if core.limits.gripper_s>settings['gripper_s']:raise ValueError('Gripper rate exceeds fixed command profile')
        core.gripper_command_profile=dict(name=gripper_command_profile,**settings)
        if feedback_framing not in ('strict','length-envelope'):raise ValueError('Unknown feedback framing')
        if hasattr(link,'kinematics_status'):link.kinematics_status['feedback_framing']=feedback_framing
        def reader():
            nonlocal feedback,fatal,feedback_count,feedback_first_ns,feedback_failure,length_envelopes,feedback_socket_options
            parser=FeedbackParser(extended=True,allow_length_envelope=feedback_framing=='length-envelope')
            trace=FeedbackTrace()
            try:
                if feedback_source:
                    while not done.wait(.008):
                        value=feedback_source()
                        with feedback_lock:feedback=value
                    return
                with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
                    # Acquire receive capacity before the controller starts
                    # streaming. The OS may clamp it; record the actual value.
                    # Priority is a request to the WLAN, never a guarantee.
                    sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
                    priority=configure_e6_socket_priority(sock)
                    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    sock.settimeout(2);sock.connect((ip,30004))
                    feedback_socket_options=dict(requested_receive_buffer_bytes=4*1024*1024,
                        effective_receive_buffer_bytes=sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF),
                        **priority)
                    sock.settimeout(.1)
                    while not done.is_set():
                        try:raw=sock.recv(16384)
                        except socket.timeout:continue
                        if not raw:parser.eof();raise ConnectionError('Feedback disconnected')
                        now=time.perf_counter_ns()
                        if hasattr(socket,'TCP_QUICKACK'):sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_QUICKACK,1)
                        trace.receive(raw,now)
                        packets=parser.feed(raw)
                        length_envelopes=parser.length_envelopes
                        for index,(packet,decoded) in enumerate(packets):
                            trace.valid(packet,decoded,now)
                            if hasattr(link,'validate_feedback'):link.validate_feedback(decoded,streamed=True)
                            feedback_count+=1
                            if feedback_first_ns is None:feedback_first_ns=now
                            value=dict(source='E6:30004 guardian',seq=feedback_count,x5_receive_monotonic_ns=now,raw_base64=base64.b64encode(packet).decode(),decoded=decoded)
                            if parser.last_transport_prefixes[index]:
                                value.update(wire_prefix_base64=base64.b64encode(parser.last_transport_prefixes[index]).decode(),wire_bytes=1448,transport_framing='explicit-duplicated-length-envelope')
                            with feedback_lock:feedback=value
            except Exception as exc:
                fatal='反馈异常: '+str(exc);link.release_pending=False;link.cancel_fault_reason=fatal;link.cancel.set()
                try:feedback_failure=trace.failure(parser,exc)
                except Exception as capture_error:feedback_failure=dict(capture_error=str(capture_error))
        def fresh():
            with feedback_lock:return feedback
        def parent_alive():return not shutdown.is_set() and time.perf_counter_ns()-parent_heartbeat.value<=250_000_000
        commissioner=Commissioner(core,link,lock,fresh,parent_alive,gripper_factory=lambda:GripperLink(gripper_command_profile))
        def publish():
            nonlocal fatal
            if hasattr(link,'connection_ready'):link.connection_ready()
            s=core.status();gripper=link.gripper and link.gripper.feedback
            value=dict(status=s,tick=tick_result,fatal=fatal,feedback_failure=feedback_failure,watchdog_failure=watchdog_failure,runtime_profile=runtime_profile,gripper=gripper,diagnostics=(core.commissioning or {}).get('samples'),can_events=link.gripper.report['events'][-20:] if link.gripper else [],
                                        guardian_alive_ns=time.perf_counter_ns(),stats=dict(motion_tx=link.motion_tx,stop_tx=link.stop_tx,can_tx=link.gripper.tx_count if link.gripper else 0,feedback_received=feedback_count,feedback_first_ns=feedback_first_ns,feedback_length_envelopes=length_envelopes,feedback_socket_options=feedback_socket_options),
                                        command_events=recent_command_events(link.events))
            try:result_queue.put_nowait(value)
            except queue.Full:link.cancel.set();core.stop('控制证据队列溢出',fault=True);fatal='控制证据队列溢出'
        def ticker():
            nonlocal media,tick_result,fatal
            deadline=time.perf_counter()
            while not done.is_set():
                try:
                    while True:media=metadata_queue.get_nowait()
                except queue.Empty:pass
                with lock:
                    s=fresh()
                    if s:
                        pre_gripper=dict(link.gripper.feedback) if link.gripper and link.gripper.feedback else None
                        tick_result=core.tick(s,media)
                        tick_result['pre_action_gripper']=pre_gripper
                        # Active ticks already exchanged CAN once. Idle monitoring holds
                        # the last goal; faults suppress all subsequent transmissions.
                        if link.gripper and not core.gripper_faulted and core.gripper_initialized and core.armed and core.recovery.state!='ready' and not (core.input and core.input['deadman']) and core.grip_target is not None:
                            try:link.grip(core.grip_target)
                            except InterruptedError:pass
                            except Exception as exc:core.gripper_faulted=True;core.stop('夹爪反馈异常: '+str(exc),fault=True)
                        publish()
                # A delayed tick must not be followed by a compressed catch-up
                # interval. Time the next cycle from the actual preceding tick
                # (or a later arm), preserving the 20 Hz maximum and budgets.
                deadline=max(deadline+.05,core.last_tick/1e9+.05 if core.last_tick is not None else 0)
                if time.perf_counter()>deadline+.025:
                    link.release_pending=False
                    link.cancel_fault_reason='守护控制循环超过预算'
                    link.cancel.set()
                    with lock:core.stop('守护控制循环超过预算',fault=True)
                    deadline=time.perf_counter()
                done.wait(max(0,deadline-time.perf_counter()))
        def watchdog():
            nonlocal watchdog_failure
            while not done.wait(.02):
                now=time.perf_counter_ns();s=fresh()
                broken=shutdown.is_set() or now-parent_heartbeat.value>250_000_000
                initializing=bool(core.commissioning and core.commissioning['state']=='running')
                stale=bool((core.armed or core.pending or initializing) and (s is None or now-s['x5_receive_monotonic_ns']>100_000_000 or not media.get('rgb') or now-media['rgb']['x5_receive_monotonic_ns']>500_000_000 or core.armed and now-core.input_ns>250_000_000 or core.last_tick is not None and now-core.last_tick>100_000_000))
                if broken or stale or fatal or media.get('parent_fatal'):
                    reason='父进程/反馈/输入看门狗停止'
                    if fatal:reason=fatal
                    elif media.get('parent_fatal'):reason='父进程观测/记录异常'
                    elif not broken:
                        if s is None or now-s['x5_receive_monotonic_ns']>100_000_000:reason='E6反馈超过100ms'
                        elif not media.get('rgb') or now-media['rgb']['x5_receive_monotonic_ns']>500_000_000:reason='RGB超过500ms'
                        elif core.armed and now-core.input_ns>250_000_000:reason='输入看门狗超时'
                        else:reason='守护控制循环超过预算'
                    link.release_pending=False
                    link.cancel_fault_reason=reason
                    link.cancel.set()
                    with lock:
                        if watchdog_failure is None:
                            watchdog_failure=dict(detected_ns=now,lock_acquired_ns=time.perf_counter_ns(),
                                parent_heartbeat_ns=parent_heartbeat.value,feedback_receive_ns=s and s['x5_receive_monotonic_ns'],
                                rgb_receive_ns=(media.get('rgb') or {}).get('x5_receive_monotonic_ns'),
                                input_receive_ns=core.input_ns,last_tick_ns=core.last_tick,
                                armed=core.armed,pending=bool(core.pending),initializing=initializing,
                                shutdown=shutdown.is_set(),fatal=fatal,parent_fatal=media.get('parent_fatal'))
                        core.stop(reason,force=initializing and core.commissioning['operation']=='enable',fault=True);publish()
                    if broken:done.set();return
        def handle(ws):
            owner=None
            try:
                if ws.request.path!='/ws/control':ws.close(1008,'Control endpoint only');return
                if ws.request.headers.get('X-E6-Control-Token')!=token:ws.close(1008,'Control session token required');return
                if ws.request.headers.get('X-E6-Control-Role')!=role:ws.close(1008,'Controller role locked at startup');return
                with lock:
                    hello=core.connect();owner=hello['session']
                ws.send(json.dumps(dict(ok=True,status=hello)))
                while not done.is_set():
                    raw=ws.recv();received_ns=time.perf_counter_ns();data=json.loads(raw)
                    if not isinstance(data,dict):raise ValueError('Object command required')
                    release=bool(data.get('operation')=='input' and core.input and core.input['deadman'] and not data.get('deadman',False)
                                 and not data.get('stop') and data.get('focused',True) and data.get('connected',True))
                    if data.get('operation')=='stop' or data.get('operation')=='input' and (data.get('stop') or not data.get('focused',True) or not data.get('connected',True) or release):
                        link.release_pending=release;link.cancel_fault_reason=None;link.cancel.set()
                    try:
                        with lock:
                            acquired_ns=time.perf_counter_ns()
                            data['session']=owner
                            op=data.get('operation')
                            if op in ('enable','initialize-gripper','diagnose-kinematics'):
                                if role!='harness' and op=='diagnose-kinematics':raise ValueError('Diagnostics are harness-only')
                                now=time.perf_counter_ns();rgb=media.get('rgb') or {};s=fresh()
                                if not rgb or now-rgb['x5_receive_monotonic_ns']>500_000_000:raise ValueError('Initialization/diagnosis requires fresh RGB')
                                d=s['decoded'] if s else {}
                                if d.get('user_index')!=0 or d.get('tool_index')!=0 or any(abs(x)>1e-8 for x in d.get('tool_configuration_raw_mm_deg',[1])) or any(abs(x)>1e-8 for x in d.get('user_configuration_raw_mm_deg',[1])):raise ValueError('Initialization/diagnosis requires zero User0/Tool0')
                                result=commissioner.start(op)
                            else:
                                s=fresh()
                                if s is None:raise ValueError('No feedback')
                                result=core.command(data,s,media.get('rgb',{}).get('x5_receive_monotonic_ns'),received_ns=received_ns)
                                if (op=='input' and not core.armed and core.input and core.input['neutral']
                                        and core.input['connected'] and core.input['focused'] and not core.input['deadman']
                                        and hasattr(link,'stage_neutral_connection')):
                                    link.stage_neutral_connection()
                                if op=='arm' and hasattr(link,'reconnect_for_arm'):link.reconnect_for_arm()
                                if core.armed and core.input and (core.input['deadman'] or core.stop_acknowledged):
                                    link.release_pending=False;link.cancel_fault_reason=None;link.cancel.clear()
                                # Resource staging may retire/recreate the
                                # socket after command() produced its status.
                                # Reply with readiness from the resulting
                                # connection, never the retired connection.
                                if hasattr(link,'connection_ready'):link.connection_ready()
                                result=core.status()
                            publish()
                        completed_ns=time.perf_counter_ns()
                        ws.send(json.dumps(dict(ok=True,status=result,request_id=data.get('request_id'),rpc_timing=dict(x5_received_ns=received_ns,
                            x5_lock_acquired_ns=acquired_ns,x5_completed_ns=completed_ns,
                            lock_wait_ms=(acquired_ns-received_ns)/1e6,processing_ms=(completed_ns-acquired_ns)/1e6))))
                    except Exception as exc:
                        with lock:
                            if not (core.recovery.waiting and str(exc)=='租约过期或已消费'):core.stop(str(exc),fault=True)
                            publish();s=core.status()
                        ws.send(json.dumps(dict(ok=False,error=str(exc),status=s,request_id=data.get('request_id'))))
            except Exception as exc:
                try:ws.send(json.dumps(dict(ok=False,error=str(exc))))
                except Exception:pass
            finally:
                if owner:
                    link.release_pending=False
                    link.cancel.set()
                    with lock:core.disconnect(owner);publish()
        def request(conn,req):
            if req.path=='/health':return conn.respond(200,json.dumps(dict(backend='hardware-control',control_stage=stage,role=role)))
            if req.path!='/ws/control':return conn.respond(403,'Control endpoint only')
        threads=[threading.Thread(target=f,daemon=True) for f in (reader,ticker,watchdog)]
        for thread in threads:thread.start()
        with serve(handle,'127.0.0.1',listen_port,process_request=request,max_size=8192,compression=None) as server:
            server_thread=threading.Thread(target=server.serve_forever,daemon=True)
            server_thread.start()
            ready.set()
            while not done.wait(.05):pass
            server.shutdown()
            server_thread.join(2)
        for thread in threads:thread.join(1)
    except Exception as exc:
        put_latest(result_queue,dict(fatal=str(exc),guardian_alive_ns=time.perf_counter_ns(),status=dict(armed=False,reason=str(exc))))
    finally:
        done.set()
        if core:
            with lock:core.stop('实机控制会话退出')
        if commissioner and commissioner.thread:commissioner.thread.join(2)
        if link:link.close()
