"""Run the unmodified Jazzy ServoNode against a declared delayed virtual plant.

Uses upstream timed JointTrajectory output and SciPy Hermite interpolation;
no E6/CAN imports or sockets. This qualifies upstream behavior, not the webpage.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.interpolate import CubicHermiteSpline
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool


def config(bundle,plugin,period):
    tree=ET.parse(bundle/'model/e6.urdf');profile=json.loads((bundle/'model/profile.json').read_text())
    assert hashlib.sha256((bundle/'model/e6.urdf').read_bytes()).hexdigest()==profile['urdf_sha256']
    for kind,directory in [('visual','meshes'),('collision','collision')]:
        for mesh in tree.findall(f'./link/{kind}/geometry/mesh'):
            mesh.set('filename',(bundle/directory/Path(mesh.get('filename')).name).as_uri())
    params=yaml.safe_load((bundle/'servo_jazzy.yaml').read_text())
    # Upstream requires its trajectory window to exceed three publish periods.
    # This first comparison retains the old 20 Hz control period; latency at
    # this setting is not a qualified production response time.
    horizon={.05:.2,.02:.1,.01:.05}[period]
    params.update(publish_period=period,max_expected_latency=horizon,move_group_name='me6_group',
        joint_topic='/qualification/actual',joint_command_in_topic='/qualification/jog',
        command_out_topic='/qualification/trajectory',status_topic='/qualification/status',
        command_in_type='speed_units',incoming_command_timeout=.25,check_collisions=True,
        lower_singularity_threshold=17.,hard_stop_singularity_threshold=30.,joint_limit_margins=[.1],
        smoothing_filter_plugin_name=('e6_reviewed_smoothing::AccelerationLimitedPlugin'
            if plugin=='ReviewedAccelerationLimitedPlugin' else 'online_signal_smoothing::'+plugin))
    limits={f'joint{i}':dict(has_velocity_limits=True,max_velocity=math.radians(2),
        has_acceleration_limits=True,max_acceleration=math.radians(4),
        has_jerk_limits=True,max_jerk=math.radians(40)) for i in range(1,7)}
    return {'/servo_node':{'ros__parameters':dict(robot_description=ET.tostring(tree.getroot(),encoding='unicode'),
        robot_description_semantic=(bundle/'model/e6.srdf').read_text(),
        robot_description_planning={'joint_limits':limits},moveit_servo=params,
        update_period=period,planning_group_name='me6_group')}}


class Plant(Node):
    def __init__(self,seed,tau,log,speed,traced=False):
        super().__init__('qualification_plant');self.q=np.deg2rad(seed);self.target=self.q.copy();self.tau=tau
        self.names=[f'joint{i}' for i in range(1,7)];self.last=time.monotonic();self.log=log
        self.speed=speed;self.feedback_enabled=False;self.traced=traced
        self.input_stamps={};self.trace_points=0;self.timed=None
        self.feedback=self.create_publisher(JointState,'/qualification/actual',1)
        self.jog=self.create_publisher(JointJog,'/qualification/jog',1)
        self.create_subscription(JointTrajectory,'/qualification/trajectory',self.trajectory,1)
        self.create_subscription(ServoStatus,'/qualification/status',self.status,1)
        self.switch=self.create_client(ServoCommandType,'/servo_node/switch_command_type')
        self.pause=self.create_client(SetBool,'/servo_node/pause_servo')
        self.curve=None;self.window=None;self.offset=0.;self.trajectories=0;self.statuses=set();self.rows=[]
        self.command=False;self.start_ns=None;self.last_jog=0;self.next_sequence=0;self.release_ns=None;self.process=None
        self.create_timer(.01,self.tick)

    def status(self,msg):self.statuses.add(msg.code)

    def trajectory(self,msg):
        if not msg.points:return
        if msg.joint_names!=self.names:raise RuntimeError('Unexpected joint order')
        t=np.array([p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in msg.points])
        q=np.array([p.positions for p in msg.points]);v=np.array([p.velocities for p in msg.points])
        if len(t)<2 or np.any(np.diff(t)<=0):return
        self.offset=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if self.traced:
            from timed_trajectory import TimedTrajectory
            stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            times_ns=[stamp_ns+p.time_from_start.sec*1_000_000_000+p.time_from_start.nanosec for p in msg.points]
            self.timed=TimedTrajectory(times_ns,q,v,msg.header.frame_id)
            for source in self.timed.sources:
                if source.kind!='s' and source.input_ns not in self.input_stamps:
                    raise RuntimeError('Upstream selected an unknown input stamp')
            self.trace_points+=len(msg.points)
        self.curve=CubicHermiteSpline(t,q,v,axis=0,extrapolate=False);self.window=(float(t[0]),float(t[-1]))
        self.trajectories+=1
        self.log.write(json.dumps(dict(event='trajectory',received_ns=time.perf_counter_ns(),
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,t=t.tolist(),q=q.tolist(),v=v.tolist(),
            trace=msg.header.frame_id))+'\n')

    def tick(self):
        now=time.monotonic();dt=now-self.last;self.last=now;stamp=self.get_clock().now()
        if self.timed is not None:
            selected=self.timed.sample(stamp.nanoseconds)
            if selected is not None:self.target=np.array(selected['joints_rad'])
        elif self.curve is not None:
            t=stamp.nanoseconds/1e9-self.offset
            if t>=self.window[0]:self.target=self.curve(min(t,self.window[1]))
        self.q+=(self.target-self.q)*(1 if self.tau==0 else 1-math.exp(-dt/self.tau))
        msg=JointState();msg.header.stamp=stamp.to_msg();msg.name=self.names;msg.position=self.q.tolist()
        if self.feedback_enabled:self.feedback.publish(msg)
        if now-self.last_jog>=.05:
            jog=JointJog();jog.header.stamp=stamp.to_msg();jog.joint_names=self.names
            jog.velocities=[0.]*5+[math.radians(self.speed) if self.command else 0.]
            self.jog.publish(jog);self.last_jog=now;self.next_sequence+=1
            self.input_stamps[stamp.nanoseconds]=self.next_sequence
            if self.start_ns and not self.command and self.release_ns is None:self.release_ns=time.perf_counter_ns()
            self.log.write(json.dumps(dict(event='input',at_ns=time.perf_counter_ns(),sequence=self.next_sequence,velocity=list(jog.velocities),
                ros_stamp_ns=stamp.nanoseconds,feedback_subscribers=self.feedback.get_subscription_count(),
                jog_subscribers=self.jog.get_subscription_count()))+'\n')
        if self.start_ns:
            row=dict(t_s=(time.perf_counter_ns()-self.start_ns)/1e9,q=self.q.tolist(),target=self.target.tolist())
            self.rows.append(row);self.log.write(json.dumps(dict(event='plant',**row))+'\n')

    def service(self,client,request):
        deadline=time.monotonic()+35
        while not client.service_is_ready():
            rclpy.spin_once(self,timeout_sec=.01)
            if self.process and self.process.poll() is not None:raise RuntimeError('Servo exited before service startup')
            if time.monotonic()>deadline:raise TimeoutError('Servo service startup')
        future=client.call_async(request)
        while not future.done():
            rclpy.spin_once(self,timeout_sec=.01)
            if time.monotonic()>deadline:raise TimeoutError('Servo service response')
        if not future.result().success:raise RuntimeError('Servo service rejected')


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--tau',type=float,required=True);p.add_argument('--plugin',default='AccelerationLimitedPlugin')
    p.add_argument('--speed-deg-s',type=float,default=.2)
    p.add_argument('--period',type=float,choices=[.05,.02,.01],default=.05)
    p.add_argument('--node-kind',choices=['original','traced'],default='original')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);params=a.output/'params.yaml'
    if not math.isfinite(a.speed_deg_s) or not 0<abs(a.speed_deg_s)<=2:raise ValueError('Virtual speed must be nonzero and at most 2 deg/s')
    shutil.copyfile(__file__,a.output/'qualify_jazzy_node.py')
    params.write_text(yaml.safe_dump(config(a.bundle,a.plugin,a.period)))
    rclpy.init();log=(a.output/'events.jsonl').open('w');node=Plant(json.loads((a.bundle/'seed.json').read_text()),a.tau,log,a.speed_deg_s,a.node_kind=='traced')
    console=(a.output/'servo.log').open('w');proc=None
    result=dict(hardware_io=False,tau_s=a.tau,plugin=a.plugin,requested_deg_s=a.speed_deg_s,servo_period_s=a.period,node_kind=a.node_kind,error=None)
    try:
        executable='/opt/ros/jazzy/lib/moveit_servo/servo_node' if a.node_kind=='original' else '/qualification/jazzy-adapter-v35/build/servo_e6_traced'
        proc=subprocess.Popen([executable,'--ros-args','--params-file',str(params)],
            stdout=console,stderr=subprocess.STDOUT)
        node.process=proc
        request=ServoCommandType.Request();request.command_type=0;node.service(node.switch,request)
        # Bring up the upstream node before starting exact static feedback.
        # ServoNode waits for a planning-scene update newer than its servo
        # thread; unchanged positions do not retrigger that scene timestamp.
        node.feedback_enabled=True
        startup=time.monotonic()+2
        while time.monotonic()<startup:rclpy.spin_once(node,timeout_sec=.01)
        node.start_ns=time.perf_counter_ns();node.command=True
        while (time.perf_counter_ns()-node.start_ns)<6_000_000_000:
            rclpy.spin_once(node,timeout_sec=.01)
            if proc.poll() is not None:raise RuntimeError('Servo exited')
        node.command=False
        end=time.monotonic()+1.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        request=SetBool.Request();request.data=True;node.service(node.pause,request)
        if node.trajectories==0:raise RuntimeError('No upstream trajectories received')
        steady=[r for r in node.rows if 2<r['t_s']<5.8]
        speed=np.polyfit([r['t_s'] for r in steady],[math.degrees(r['q'][5]) for r in steady],1)[0]
        final=[r for r in node.rows if r['t_s']>7]
        final_excursion=np.ptp(np.array([r['q'] for r in final]),axis=0)
        result.update(actual_deg_s=float(speed),tracking_ratio=float(speed/a.speed_deg_s),trajectories=node.trajectories,statuses=sorted(node.statuses),
            trace_points=node.trace_points,
            release_input_t_s=(node.release_ns-node.start_ns)/1e9,
            final_half_second_excursion_deg=np.rad2deg(final_excursion).tolist(),
            virtual_speed_within_five_percent=bool(.95<=speed/a.speed_deg_s<=1.05))
        (a.output/'loaded-libraries.txt').write_text(Path('/proc') .joinpath(str(proc.pid),'maps').read_text())
    except Exception as exc:result['error']=str(exc)
    finally:
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:proc.wait(timeout=3)
                except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
        result['exit_code']=proc.returncode if proc else None;node.destroy_node();rclpy.shutdown();console.close();log.close()
        if result['exit_code']!=0 and result['error'] is None:result['error']='Servo did not exit normally'
        (a.output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    return int(result['error'] is not None)


if __name__=='__main__':raise SystemExit(main())
