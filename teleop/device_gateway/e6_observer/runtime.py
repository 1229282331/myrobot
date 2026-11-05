"""Own acquisition threads; immutable samples cross into the 20 Hz publisher."""
import base64
from concurrent.futures import ThreadPoolExecutor
import fcntl
import hashlib
import json
import os
import queue
import re
import socket
import subprocess
import threading
import time
import zlib
from collections import deque
from .protocol import FeedbackParser, READ_QUERIES, assemble
from .archive import ArchiveStore
from .process_lifecycle import join_draining_results


class Hub:
    def __init__(self, robot_ip, camera_tools, instance, control_stage=None, control_role='harness', control_token=None, control_limits=None, kinematics_profile=None,feedback_framing='strict',recovery_policy='manual',gripper_command_profile='plan32'):
        self.robot_ip, self.camera_tools, self.instance = robot_ip, camera_tools, instance
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest, self.threads, self.sockets = {}, [], []
        self.stats = {n:dict(received=0, source_index_gaps=0, source_time_reversals=0) for n in ("robot","rgb","depth")}
        self.stats.update(can_tx_count=0, preview_discarded=0, acquisition_queue_overflows=0, fatal=None)
        self.acquisition_lock = None
        self.archive = ArchiveStore('native_archives')
        self.control_stage,self.control_role,self.control_token=control_stage,control_role,control_token
        self.control_limits=control_limits or {}
        self.kinematics_profile=kinematics_profile
        self.feedback_framing=feedback_framing
        if recovery_policy not in ('manual','bounded'):raise ValueError('Unknown recovery policy')
        self.recovery_policy=recovery_policy
        from .gripper_profile import gripper_profile
        gripper_profile(gripper_command_profile)
        if not control_stage and gripper_command_profile!='plan32':raise ValueError('Observation has no gripper command configuration')
        self.gripper_command_profile=gripper_command_profile
        # Preview only. Native RGB/depth acquisition, archive bytes and the
        # guardian's 20 Hz source selection are unchanged.
        self.preview_profile=dict(rgb=[480,270],depth=[240,180],jpeg_quality=55) if control_stage else dict(rgb=[640,360],depth=[320,240],jpeg_quality=70)
        self.guardian=None;self.control_latest=None;self.selection_cache={'rgb':{},'depth':{}}
        self.control_backlog=deque();self.control_sampling=False;self.last_control_tick=None
        self.control_sampling_started_ns=None
        self.control_sampling_owner=None

    def begin_control_sampling(self,owner):
        with self.lock:
            if self.control_sampling_owner is not None:raise ValueError('Control sampling already has a consumer')
            self.control_sampling_owner=owner
            self.control_sampling_started_ns=time.perf_counter_ns()
            self.control_backlog.clear();self.control_sampling=True

    def end_control_sampling(self,owner):
        with self.lock:
            if self.control_sampling_owner is owner:
                self.control_sampling=False;self.control_sampling_owner=None
                self.control_backlog.clear()

    def publish(self, name, value):
        with self.lock:
            stat = self.stats[name]
            old = self.latest.get(name)
            if old and value.get("device_frame_id") is not None:
                diff = value["device_frame_id"]-old["device_frame_id"]
                if diff <= 0:
                    raise ValueError(name+" frame ID reset/reversal; restart acquisition explicitly")
                stat["source_index_gaps"] += max(0,diff-1)
            stat["received"] += 1
            now=value['x5_receive_monotonic_ns']
            if 'first_receive_ns' not in stat:stat['first_receive_ns']=now
            stat['last_receive_ns']=now
            stat['actual_fps']=(stat['received']-1)*1e9/max(1,now-stat['first_receive_ns'])
            value["seq"] = value.get('seq',stat["received"])
            self.latest[name] = value
            if self.control_stage and name in self.selection_cache:
                cache=self.selection_cache[name];cache[value['device_frame_id']]=value
                while len(cache)>64:cache.pop(next(iter(cache)))

    def fail(self, name, exc):
        with self.lock:
            self.stats[name+"_error"] = str(exc)
            if name in self.latest:
                self.latest[name] = dict(self.latest[name], error=str(exc))
            else:
                self.latest[name] = dict(x5_receive_monotonic_ns=time.perf_counter_ns(),error=str(exc),source=name)

    def snapshot(self, seq, selected_media=None, control_sample=False):
        if self.control_stage and control_sample:
            deadline=time.monotonic()+.1
            while not self.stop.is_set() and time.monotonic()<deadline:
                with self.lock:
                    if self.control_backlog:break
                self.stop.wait(.002)
        with self.lock:
            latest=dict(self.latest)
            control=self.control_latest
            if self.control_stage and control_sample and self.control_backlog:
                control=self.control_backlog.popleft()
            if self.control_stage and control and control.get('tick'):
                tick=control['tick'];latest['robot']=tick['pre_action_robot']
                for name,spec in tick['selected_media'].items():
                    if name in self.selection_cache:latest[name]=self.selection_cache[name].get(spec.get('device_frame_id'))
            if selected_media is not None:latest.update(selected_media)
            frame=assemble(latest, json.loads(json.dumps(self.stats)), self.instance, seq)
            frame['stats']['control_sampling_debug']=dict(backlog=len(self.control_backlog),start_ns=self.control_sampling_started_ns,latest_tick=(self.control_latest or {}).get('tick',{}).get('control_tick_id')) if self.control_stage else None
            if self.control_stage:
                frame.update(schema_version=3,backend='hardware-control',control_stage=self.control_stage,
                             control_status=control.get('status') if control else {'armed':False,'reason':'守护进程启动中'},
                             control=control, candidate_action=None)
                frame['capabilities'].update(control=True,operator_control=self.control_role=='operator',controller_role=self.control_role)
                if control and control.get('tick'):
                    tick=control['tick']
                    frame.update(input=tick['input'],effective_action=tick['effective_action'],candidate_action=tick['candidate_action'])
                    frame['observation'].update(control_tick_id=tick['control_tick_id'],control_tick_monotonic_ns=tick['control_tick_monotonic_ns'])
                    frame['status']=control['status']
                if control and control.get('tick',{}).get('pre_action_gripper'):
                    g=dict(control['tick']['pre_action_gripper']);g['valid']=0<=control['tick']['control_tick_monotonic_ns']-g['x5_receive_monotonic_ns']<=100_000_000
                    frame['sources']['gripper']=g;frame['observation']['gripper']=g.get('opening') if g['valid'] else None
                    frame['capabilities']['gripper_telemetry']=g['valid']
                health=self.control_latest
                if not health or health.get('fatal') or time.perf_counter_ns()-health.get('guardian_alive_ns',0)>150_000_000:
                    frame['observation']['valid']=False;frame['stats']['fatal']=(control or {}).get('fatal') or '控制守护进程未更新'
        frame['stats']['archive']=self.archive.status()
        if frame['stats']['archive']['error']:frame['stats']['fatal']=frame['stats']['archive']['error']
        frame['capabilities'].update(deferred_media=True,preview=True)
        frame['capabilities']['preview_configuration']=self.preview_profile
        frame['transport_profile']='latest-preview-deferred-originals'
        return frame

    def start(self):
        self.acquisition_lock = open("/tmp/e6-observer-acquisition.lock", "a")
        fcntl.flock(self.acquisition_lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        if self.control_stage:
            import multiprocessing
            from .control_guardian import run_guardian
            from .control_core import ControlLimits
            from dataclasses import asdict
            context=multiprocessing.get_context('spawn')
            self.control_metadata=context.Queue(maxsize=1);self.control_results=context.Queue(maxsize=128)
            self.parent_heartbeat=context.Value('q',time.perf_counter_ns());self.guardian_shutdown=context.Event();ready=context.Event()
            limits=asdict(ControlLimits());limits.update(self.control_limits)
            # Native SDK/OpenCV startup may hold the parent's interpreter.
            # Complete it before starting the independent control lifetime;
            # never give an active guardian a startup watchdog exemption.
            for target in (self.camera,self.can_status):
                thread=threading.Thread(target=target,daemon=True)
                thread.start();self.threads.append(thread)
            self.wait_camera_ready()
            bridge=threading.Thread(target=self.control_bridge,daemon=True)
            bridge.start();self.threads.append(bridge)
            self.guardian=context.Process(target=run_guardian,args=(self.robot_ip,self.control_stage,self.control_role,self.control_token,limits,self.control_metadata,self.control_results,self.parent_heartbeat,self.guardian_shutdown,ready),kwargs={'kinematics_profile':self.kinematics_profile,'feedback_framing':self.feedback_framing,'recovery_policy':self.recovery_policy,'gripper_command_profile':self.gripper_command_profile})
            self.guardian.start()
            if not ready.wait(8):raise RuntimeError('控制守护进程未启动；拒绝采图或接管设备')
        targets=() if self.control_stage else (self.robot,self.dashboard,self.camera,self.can_status)
        for target in targets:
            thread = threading.Thread(target=target, daemon=True)
            thread.start(); self.threads.append(thread)

    def wait_camera_ready(self,timeout=8):
        begin=time.perf_counter_ns();deadline=time.monotonic()+timeout
        while not self.stop.is_set() and time.monotonic()<deadline:
            with self.lock:
                error=self.stats.get('fatal') or self.stats.get('rgb_error') or self.stats.get('depth_error')
                if error:raise RuntimeError('相机启动失败；控制守护未启动: '+str(error))
                rgb=self.latest.get('rgb') or {}
                if rgb and not rgb.get('error') and time.perf_counter_ns()-rgb['x5_receive_monotonic_ns']<=500_000_000:
                    self.stats['control_startup']=dict(camera_ready_before_guardian=True,camera_startup_ms=(time.perf_counter_ns()-begin)/1e6)
                    return
            self.stop.wait(.01)
        raise RuntimeError('相机启动未就绪；控制守护未启动')

    def close(self):
        self.stop.set()
        error=None
        if self.guardian:
            self.guardian_shutdown.set()
            if not join_draining_results(self.guardian,self.control_results):error=RuntimeError('Guardian退出未确认；不得宣称机械臂已停止')
        for sock in tuple(self.sockets):
            try: sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
        for thread in self.threads: thread.join(5)
        self.archive.close()
        if self.acquisition_lock: self.acquisition_lock.close()
        if error:raise error

    def control_bridge(self):
        from .control_guardian import put_latest
        while not self.stop.wait(.025):
            self.parent_heartbeat.value=time.perf_counter_ns()
            with self.lock:
                metadata={n:{k:v for k,v in s.items() if k not in ('jpeg','uint16_raw','preview_jpeg','metadata')} for n,s in self.latest.items() if n in ('rgb','depth')}
                metadata['parent_fatal']=self.stats.get('fatal')
            put_latest(self.control_metadata,metadata)
            try:
                while True:
                    value=self.control_results.get_nowait()
                    value['command_events']=value.get('command_events',[])[-20:]
                    with self.lock:
                        self.control_latest=value
                        tick=(value.get('tick') or {}).get('control_tick_id')
                        if tick is not None and tick!=self.last_control_tick:
                            if self.control_sampling and value['tick']['control_tick_monotonic_ns']>=self.control_sampling_started_ns:
                                if len(self.control_backlog)>=32:
                                    self.stats['fatal']='Control sampling queue overflow; no action catch-up'
                                else:self.control_backlog.append(value)
                            self.last_control_tick=tick
                    robot=(value.get('tick') or {}).get('pre_action_robot')
                    if robot and (self.latest.get('robot') or {}).get('seq')!=robot.get('seq'):
                        self.publish('robot',dict(robot))
                        native=value.get('stats',{})
                        if native.get('feedback_first_ns'):
                            with self.lock:self.stats['robot']['actual_fps']=(native['feedback_received']-1)*1e9/max(1,robot['x5_receive_monotonic_ns']-native['feedback_first_ns'])
            except queue.Empty:pass

    def robot(self):
        parser = FeedbackParser(extended=bool(self.control_stage))
        try:
            with socket.create_connection((self.robot_ip,30004),timeout=2) as sock:
                self.sockets.append(sock); sock.settimeout(.2)
                while not self.stop.is_set():
                    try: chunk = sock.recv(16384)
                    except socket.timeout: continue
                    if not chunk: parser.eof(); raise ConnectionError("E6反馈连接关闭")
                    now = time.perf_counter_ns()
                    for packet, decoded in parser.feed(chunk):
                        self.publish("robot",dict(source="E6:30004",x5_receive_monotonic_ns=now,
                                                  raw_base64=base64.b64encode(packet).decode(),decoded=decoded))
        except Exception as exc:
            if not self.stop.is_set(): self.fail("robot",exc)

    def dashboard(self):
        try:
            with socket.create_connection((self.robot_ip,29999),timeout=2) as sock:
                self.sockets.append(sock); sock.settimeout(1)
                while not self.stop.is_set():
                    replies = []
                    for command in READ_QUERIES:
                        sock.sendall(command.encode("ascii"))
                        reply = bytearray()
                        while b";" not in reply:
                            chunk=sock.recv(4096)
                            if not chunk: raise ConnectionError("Dashboard closed")
                            reply.extend(chunk)
                            if len(reply)>65536: raise ValueError("Dashboard reply too long")
                        text=reply.decode("utf-8").strip()
                        match=re.fullmatch(r"0,\{(.*)\},"+re.escape(command)+r";",text,re.DOTALL)
                        if not match: raise ValueError("Dashboard query rejected or wrong framing: "+text[:200])
                        replies.append(dict(query=command,payload=match.group(1),raw=text,x5_receive_monotonic_ns=time.perf_counter_ns()))
                    with self.lock: self.stats["dashboard"]=replies
                    self.stop.wait(1)
        except Exception as exc:
            if not self.stop.is_set(): self.fail("dashboard",exc)

    def can_status(self):
        while not self.stop.is_set():
            try:
                proc=subprocess.run(["ip","-json","-details","-statistics","link","show","can1"],capture_output=True,text=True,timeout=2)
                value=json.loads(proc.stdout) if proc.returncode==0 else dict(error=proc.stderr.strip())
                with self.lock: self.stats["can_interface"]=value
            except Exception as exc:
                with self.lock: self.stats["can_interface"]={"error":str(exc)}
            self.stop.wait(2)

    def camera(self):
        import sys
        sys.path.insert(0,self.camera_tools)
        camera = None
        inbox=queue.Queue(maxsize=24)
        done=threading.Event()
        try:
            import cv2
            # Avoid OpenCV spawning a large native thread pool on the small X5.
            cv2.setNumThreads(1)
            import numpy as np
            from camera import Camera,decode_color,decode_depth
            from .camera_frames import build_sample
            from rdk_healthcheck import ordered_frames
            # Reject another process already occupying the known video devices.
            import glob
            nodes=glob.glob("/dev/video*")
            if nodes:
                occupied=subprocess.run(["fuser",*nodes],capture_output=True,timeout=2)
                if occupied.returncode==0: raise RuntimeError("Camera/video nodes already occupied; refusing takeover")
            camera=Camera('both',serial='AY4A1520111',profiles={'color':(1280,720,5,30),'depth':(640,480,12,30)},align_mode=0,uvc_pts=True)
            with self.lock:
                self.stats['camera_identity']=camera.info
                self.stats['camera_profiles']=camera.selected
                self.stats['camera_alignment']='native unaligned RGB/depth'
            decode_report={'decode_pending_max':0}
            def receiver():
                try:
                    while not self.stop.is_set():
                        for meta,data in camera.read():
                            try: inbox.put_nowait((meta,data))
                            except queue.Full:
                                with self.lock:
                                    self.stats['acquisition_queue_overflows']+=1
                                    self.stats['fatal']='Camera acquisition queue overflow'
                                raise RuntimeError('Camera acquisition queue overflow')
                except Exception as exc:
                    if not self.stop.is_set(): self.fail('rgb',exc); self.fail('depth',exc)
                finally: done.set()
            thread=threading.Thread(target=receiver,daemon=True); thread.start()
            def process_sample(meta,data,decode_rgb=True):
                if not decode_rgb:raise ValueError('All native RGB frames must be decoded')
                begin=time.perf_counter_ns()
                value=build_sample(meta,data,self.preview_profile,decode_color,decode_depth)
                return value,begin,time.perf_counter_ns()
            with ThreadPoolExecutor(max_workers=2) as pool:
                # Resize, depth preview, JPEG and hashing run in the same
                # bounded workers as decoding. Serial preview processing
                # previously throttled the 30+30 fps acquisition consumer.
                for meta,value,begin,end in ordered_frames(inbox,done,pool,2,decode_report,1,decoder=process_sample):
                    if self.stop.is_set(): break
                    name='rgb' if meta['stream']=='color' else 'depth'
                    self.publish(name,value)
                    with self.lock:
                        self.stats['decode_pending_max']=decode_report['decode_pending_max']
                        self.stats[name+'_processing_ms']=(end-begin)/1e6
                        self.stats['camera_processing_scope']='two bounded workers: decode, preview and hash; all native frames'
            done.set(); thread.join(2)
        except Exception as exc:
            if not self.stop.is_set(): self.fail('rgb',exc); self.fail('depth',exc)
        finally:
            if camera: camera.close()
