"""ROS/KDL-only subprocess. No device SDK, CAN objects or network control API.

The host owns enable, Stop, input authorization and the 20 Hz ServoJ sender.
Here we take the latest complete point of the official 100 Hz trajectory.
ServoJ is an immediate position endpoint consumer, not a ROS trajectory
controller: waiting for the upstream future stamp would add its transport
allowance a second time. Original stamps and full provenance remain recorded.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from .latest_pipe import LatestJsonPipe
from .moveit_config import load_jazzy_bundle
from .motion_profiles import get_profile
from .timed_trajectory import TimedTrajectory
from .runtime_timing import RuntimeTiming


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TwistStamped
    from control_msgs.msg import JointJog
    from trajectory_msgs.msg import JointTrajectory
    from moveit_msgs.msg import ServoStatus
    from moveit_msgs.srv import ServoCommandType
    from std_srvs.srv import SetBool
    parser=argparse.ArgumentParser();parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--session',type=Path,required=True);parser.add_argument('--device-period',type=float,default=.05)
    parser.add_argument('--motion-profile',default='commissioning')
    parser.add_argument('--tcp-profile',default='flange')
    args=parser.parse_args();args.session.mkdir(parents=True,exist_ok=True)
    params,profile=load_jazzy_bundle(args.bundle,args.device_period,args.motion_profile,args.tcp_profile)
    config=args.session/'params.yaml';config.write_text(yaml.safe_dump({'/servo_node':{'ros__parameters':params}}))
    pipe=LatestJsonPipe(sys.stdin.fileno(),sys.stdout.fileno());logs=[];servo=None;kdl=None;node=None
    clock=time.perf_counter_ns;stop=[False]
    timing=RuntimeTiming();timing.install()
    def interrupted(*_):stop[0]=True
    # Logging must never enter the stdout JSON transport.
    rclpy.init();signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    result={'error':None};latest_info=dict(ready=False,phase='starting',error=None,status=None)
    try:
        for name in ('servo','kdl'):logs.append((args.session/(name+'.log')).open('w'))
        binary=args.bundle/'jazzy-adapter-v35/build'
        servo=subprocess.Popen([str(binary/'servo_e6_traced'),'--ros-args','--params-file',str(config)],
                               stdout=logs[0],stderr=subprocess.STDOUT)
        kdl=subprocess.Popen([str(binary/'kdl_model_probe'),str(args.bundle/'model/e6.urdf'),
                              'controller_base','controller_flange'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                             stderr=logs[1],text=True,bufsize=1)
        class Adapter(Node):
            def __init__(self):
                super().__init__('e6_jazzy_adapter')
                self.names=[f'joint{i}' for i in range(1,7)]
                self.feedback=self.create_publisher(JointState,'/e6_adapter/joint_states',1)
                self.jog=self.create_publisher(JointJog,'/e6_adapter/jog',1)
                self.twist=self.create_publisher(TwistStamped,'/e6_adapter/twist',1)
                self.create_subscription(JointTrajectory,'/e6_adapter/candidates',self.trajectory,1)
                self.create_subscription(ServoStatus,'/e6_adapter/status',self.status,1)
                self.pause=self.create_client(SetBool,'/servo_node/pause_servo')
                self.switch=self.create_client(ServoCommandType,'/servo_node/switch_command_type')
                self.incoming=None;self.actual=None;self.command=None;self.epoch=0
                self.future=None;self.operation=None;self.paused=None;self.active_epoch=None;self.active_kind=None
                self.operation_epoch=None;self.pause_ns=0;self.next_publish=0;self.deadline=time.monotonic()+35
                self.published={};self.timed=None;self.last_trajectory_ns=0;self.counter=0;self.status_code=None
                self.model_error=None;self.last_model_ns=0;self.feedback_started=False;self.candidate=None
                self.boot_complete=False
                self.sample_counts={};self.last_sample=None;self.last_trajectory=None
                self.last_actual_fk_ms=None;self.pump_started_ns=0
                self.first_generated_epoch=None;self.first_generated_ns=None
                self.trajectory_rejections=0;self.last_rejection=None;self.create_timer(.01,self.pump)
                # The single-threaded ROS executor coalesces these wakeups.
                # They only advance service transitions / the first candidate,
                # never the 20 Hz publisher or the independent device sender.
                self.wake=self.create_guard_condition(self.pump)

            def fk(self,q):
                kdl.stdin.write(' '.join(map(str,q))+'\n');kdl.stdin.flush();line=kdl.stdout.readline()
                if not line:raise RuntimeError('KDL worker closed')
                return json.loads(line)

            def status(self,msg):self.status_code=msg.code

            def call(self,operation,value):
                if self.future is not None:return
                client=self.pause if operation=='pause' else self.switch
                if not client.service_is_ready():return
                request=SetBool.Request() if operation=='pause' else ServoCommandType.Request()
                if operation=='pause':request.data=value
                else:request.command_type=value
                self.future=client.call_async(request);self.operation=(operation,value)
                self.operation_epoch=self.epoch;self.deadline=time.monotonic()+5
                self.future.add_done_callback(lambda _:self.wake.trigger())

            def service_progress(self):
                if self.future is not None and self.future.done():
                    reply=self.future.result()
                    if not reply or not reply.success:raise RuntimeError('Servo service rejected '+str(self.operation))
                    operation,value=self.operation
                    if operation=='pause':
                        self.paused=value
                        if value:
                            self.pause_ns=clock();self.active_epoch=None;self.timed=None;self.published.clear()
                        else:self.active_epoch=self.operation_epoch
                    else:self.active_kind=value
                    self.future=None;self.operation=None
                if (self.future is not None or self.paused is None) and time.monotonic()>self.deadline:
                    raise TimeoutError('Servo service startup/transition')

            def publish(self,command):
                intent=command['intent'];stamp=self.get_clock().now().to_msg();stamp_ns=stamp.sec*10**9+stamp.nanosec
                kind='j' if any(intent['joint_rad_s']) else 't'
                if kind=='j':
                    msg=JointJog();msg.header.stamp=stamp;msg.joint_names=self.names;msg.velocities=intent['joint_rad_s']
                    self.jog.publish(msg)
                else:
                    msg=TwistStamped();msg.header.stamp=stamp
                    msg.header.frame_id=profile['tcp']['frame'] if intent['frame']=='tool' else 'controller_base'
                    for i,k in enumerate('xyz'):
                        setattr(msg.twist.linear,k,intent['linear_m_s'][i]);setattr(msg.twist.angular,k,intent['angular_rad_s'][i])
                    self.twist.publish(msg)
                self.published[stamp_ns]=dict(sequence=command['sequence'],epoch=command['epoch'],kind=kind,
                    ros_stamp_ns=stamp_ns,at_ns=clock(),intent=intent,observation_received_ns=self.actual['received_ns'])
                self.published={t:v for t,v in self.published.items() if clock()-v['at_ns']<2_000_000_000}
                self.next_publish=clock()+int(args.device_period*1e9)

            def trajectory(self,msg):
                if self.paused is not False or self.active_epoch!=self.epoch or self.future is not None:return
                if msg.joint_names!=self.names:raise ValueError('Unexpected trajectory joint order')
                if len(msg.points)<2:return
                stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                times=[stamp+p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in msg.points]
                trajectory=TimedTrajectory(times,[p.positions for p in msg.points],[p.velocities for p in msg.points],msg.header.frame_id)
                for source in trajectory.sources:
                    if source.kind=='s':continue
                    record=self.published.get(source.input_ns)
                    if (not record or record['epoch']!=self.epoch or source.kind!=record['kind']):
                        self.trajectory_rejections+=1;self.last_rejection='old, unknown or halted input source';return
                self.timed=trajectory;self.last_trajectory_ns=clock()
                self.last_trajectory=dict(received_ns=self.last_trajectory_ns,first_ros_ns=times[0],last_ros_ns=times[-1])
                if self.first_generated_epoch!=self.epoch:self.wake.trigger()

            def pump(self):
                timing.begin()
                try:self._pump()
                finally:timing.end()

            def _pump(self):
                nonlocal latest_info
                if servo.poll() is not None or kdl.poll() is not None:raise RuntimeError('Servo/KDL child exited')
                now=clock();self.pump_started_ns=now;incoming=pipe.receive()
                timing.mark('receive_latest_pipe')
                if incoming:self.incoming=incoming
                self.service_progress()
                timing.mark('service_progress')
                if not self.incoming:
                    pipe.offer(dict(info=latest_info,candidate=None));pipe.flush();return
                self.command=self.incoming['command'];self.epoch=self.incoming['epoch'];self.actual=self.incoming['actual']
                # Receiving/decoding may finish after the host sampled a new
                # command. Compare freshness at the consumer boundary, never
                # against the timestamp taken before reading that command.
                now=clock()
                s=self.actual['status'];received=self.actual.get('received_ns')
                fresh=bool(s and self.actual['connected'] and received and 0<=now-received<=100_000_000)
                # Start exact static feedback only after the Servo service exists.
                # This avoids the upstream planning-scene startup timestamp race.
                if fresh and self.pause.service_is_ready():
                    msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=self.names
                    msg.position=np.deg2rad(s['joint_state']['q_actual']).tolist()
                    timing.mark('prepare_feedback')
                    self.feedback.publish(msg)
                    timing.mark('publish_feedback')
                    self.feedback_started=True
                    if now-self.last_model_ns>=50_000_000:
                        fk_started=clock();model=self.fk(msg.position);self.last_actual_fk_ms=(clock()-fk_started)/1e6;p=s['tool_vector_actual']
                        dp=float(np.linalg.norm(np.array(model['position_m'])*1000-[p[k] for k in ('x','y','z')]))
                        dr=math.degrees((Rotation.from_quat(model['quaternion_xyzw']).inv()*
                            Rotation.from_euler('xyz',[p[k] for k in ('rx','ry','rz')],degrees=True)).magnitude())
                        self.model_error=dict(position_mm=dp,orientation_deg=dr);self.last_model_ns=now
                    timing.mark('actual_fk_and_alignment')
                aligned=bool(fresh and self.model_error and self.model_error['position_mm']<=1 and self.model_error['orientation_deg']<=1)
                command=self.command
                active=bool(self.boot_complete and command and aligned and command['epoch']==self.epoch and 0<=now-command['received_ns']<=250_000_000)
                wanted_kind=0 if command and any(command['intent']['joint_rad_s']) else 1
                if not self.boot_complete:
                    # A service being reachable does not mean Servo's state
                    # initialization loop is running. Observe its own status
                    # without publishing ANY input, then pause before readiness.
                    if self.paused is None:self.call('pause',True)
                    elif aligned and self.future is None:
                        if self.status_code is not None and self.paused:self.boot_complete=True
                        elif self.paused:self.call('pause',False)
                        elif self.status_code is not None:self.call('pause',True)
                elif not active or self.active_epoch!=self.epoch or self.active_kind!=wanted_kind:
                    self.candidate=None;self.timed=None
                    if not self.paused:self.call('pause',True)
                    elif active and self.future is None and now-self.pause_ns>=50_000_000:
                        if self.active_kind!=wanted_kind:self.call('switch',wanted_kind)
                        else:self.publish(command);self.call('pause',False)
                elif self.future is None:
                    if now>=self.next_publish:self.publish(command)
                    timing.mark('publish_command_or_transition')
                    self.candidate=self.sample(now)
                    timing.mark('sample_candidate_and_fk')
                latest_info=dict(ready=self.boot_complete and aligned,phase='active' if active and self.active_epoch==self.epoch else 'paused' if self.boot_complete else 'starting',
                    error=None if aligned else 'Model/feedback not yet aligned',status=self.status_code,model_error=self.model_error,
                    urdf_sha256=profile['urdf_sha256'],servo_pid=servo.pid,kdl_pid=kdl.pid,tcp=profile['tcp'],
                    trajectory_rejections=self.trajectory_rejections,last_rejection=self.last_rejection,
                    transition=dict(active=active,command_age_ms=None if not command else (now-command['received_ns'])/1e6,
                        epoch=self.epoch,active_epoch=self.active_epoch,paused=self.paused,
                        operation=self.operation,operation_epoch=self.operation_epoch,
                        active_kind=self.active_kind,wanted_kind=wanted_kind),
                    sampling=dict(counts=dict(self.sample_counts),last=self.last_sample,trajectory=self.last_trajectory,
                        actual_fk_ms=self.last_actual_fk_ms,pump_ms=(clock()-self.pump_started_ns)/1e6,
                        runtime_timing=timing.snapshot()),
                    shared_clock='same X5 kernel CLOCK_MONOTONIC',candidate_selection='latest_complete_upstream_point',
                    motion_profile=get_profile(args.motion_profile).description())
                timing.mark('prepare_info')
                pipe.offer(dict(info=latest_info,candidate=self.candidate))
                timing.mark('encode_latest_pipe')
                pipe.flush()
                timing.mark('flush_latest_pipe')

            def sample(self,now):
                started=clock();ros_now_ns=self.get_clock().now().nanoseconds
                sample_ros_ns=self.timed.times[-1] if self.timed else None
                def report(reason,**extra):
                    self.sample_counts[reason]=self.sample_counts.get(reason,0)+1
                    self.last_sample=dict(reason=reason,at_ns=started,pump_to_sample_ms=(started-now)/1e6,
                        trajectory_age_ms=(started-self.last_trajectory_ns)/1e6 if self.last_trajectory_ns else None,
                        sample_ros_ns=sample_ros_ns,ros_now_ns=ros_now_ns,**extra)
                if not self.timed:report('no_trajectory');return None
                if now-self.last_trajectory_ns>50_000_000:report('trajectory_stale');return None
                # composeTrajectoryMessage omits the newest raw point until
                # its predecessor's velocity is complete. Select the last
                # published point; never extrapolate or use the omitted point.
                sampled=self.timed.latest_complete()
                if sampled is None:
                    report('outside_window',first_ros_ns=self.timed.times[0],last_ros_ns=self.timed.times[-1]);return None
                sources=[self.published[s.input_ns] for s in sampled['bracket_sources'] if s.kind!='s']
                if not sources:report('no_input_source');return None
                source=sources[-1]
                if now-source['at_ns']>250_000_000:report('source_stale');return None
                q=sampled['joints_rad'];fk_started=clock();model=self.fk(q);self.counter+=1
                completed=clock();report('produced',id=self.counter,target_fk_ms=(completed-fk_started)/1e6,
                    sample_work_ms=(completed-started)/1e6,deadline_remaining_ms=(now+25_000_000-completed)/1e6)
                if self.first_generated_epoch!=self.epoch:
                    self.first_generated_epoch=self.epoch;self.first_generated_ns=completed
                history=[dict(asdict(s),input=self.published.get(s.input_ns)) for s in sampled['window_sources']]
                return dict(id=self.counter,epoch=self.epoch,received_ns=now,valid_until_ns=now+25_000_000,
                    sample_started_ns=started,computed_ns=completed,epoch_first_computed_ns=self.first_generated_ns,
                    joints_rad=q,position_m=model['position_m'],quaternion_xyzw=model['quaternion_xyzw'],status=self.status_code,
                    sample_ros_ns=sampled['sample_time_ns'],bracket_ros_ns=sampled['bracket_times_ns'],
                    input_sequence=source['sequence'],source_input=source,trajectory_sources=history,
                    input_association='exact bracket sources; full smoothing window retained',pre_action_received_ns=self.actual['received_ns'])

        node=Adapter()
        while not stop[0]:rclpy.spin_once(node,timeout_sec=.01)
    except EOFError:pass
    except Exception as exc:
        import traceback
        result['error']=str(exc);result['traceback']=traceback.format_exc()
        try:
            pipe.offer(dict(info=dict(latest_info,ready=False,phase='failed',error=str(exc)),candidate=None));pipe.flush()
        except Exception:pass
        print(result['traceback'],file=sys.stderr)
    finally:
        timing.close()
        if node:node.destroy_node()
        rclpy.shutdown()
        for name,proc in [('servo',servo),('kdl',kdl)]:
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:proc.wait(timeout=1)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=1)
            result[name+'_exit']=proc.returncode if proc else None
        for log in logs:log.close()
        (args.session/'exit.json').write_text(json.dumps(result,indent=2))
    return int(result['error'] is not None)


if __name__=='__main__':raise SystemExit(main())
