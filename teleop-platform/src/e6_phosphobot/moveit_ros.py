"""ROS-side worker: official Servo + KDL, with no E6 or CAN command objects."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .moveit_config import load_bundle


def run(bridge):
    # Isolated localhost DDS domain. No robot driver/bringup is launched.
    os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_DOMAIN_ID']='87'
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TwistStamped
    from control_msgs.msg import JointJog
    from trajectory_msgs.msg import JointTrajectory
    from std_msgs.msg import Int8
    from std_srvs.srv import Trigger
    import yaml

    bundle=bridge.bundle.resolve();params,profile=load_bundle(bundle,bridge.period)
    runtime=bundle/'sessions'/str(time.time_ns());runtime.mkdir(parents=True,exist_ok=False)
    with bridge.lock:bridge.info['session']=str(runtime)
    config=runtime/'params.yaml';config.write_text(yaml.safe_dump({'/servo_node':{'ros__parameters':params}}))
    context=Context();rclpy.init(context=context);node=None;executor=None
    log=(runtime/'servo.log').open('w');kdl_log=(runtime/'kdl.log').open('w')
    try:
        bridge.kdl_process=subprocess.Popen([str(bundle/'kdl_model_probe'),str(bundle/'model/e6.urdf'),
            'controller_base','controller_flange'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=kdl_log,text=True,bufsize=1)
        # This fixed-source build differs only in the provenance metadata patch.
        servo_env=dict(os.environ,LD_LIBRARY_PATH=str(bundle/'servo-build')+':'+os.environ.get('LD_LIBRARY_PATH',''))
        bridge.process=subprocess.Popen([str(bundle/'servo-build/servo_node_main'),'--ros-args',
            '--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=servo_env)

        class Adapter(Node):
            def __init__(self):
                super().__init__('e6_moveit_adapter',context=context)
                self.names=[f'joint{i}' for i in range(1,7)]
                self.feedback=self.create_publisher(JointState,'/e6_adapter/joint_states',1)
                self.twist=self.create_publisher(TwistStamped,'/e6_adapter/twist',1)
                self.jog=self.create_publisher(JointJog,'/e6_adapter/jog',1)
                self.create_subscription(JointTrajectory,'/e6_adapter/candidates',self.candidate,1)
                self.create_subscription(Int8,'/servo_node/status',self.status,1)
                self.service_clients={name:self.create_client(Trigger,'/servo_node/'+name+'_servo')
                              for name in ('start','pause','unpause')}
                self.future=None;self.operation=None;self.started=False;self.paused=False
                self.pause_ns=0;self.pending_epoch=None;self.active_epoch=None
                self.next_publish=0;self.counter=0;self.published=[];self.actual=None;self.model=None
                self.model_error=None;self.last_model_ns=0;self.status_code=None
                self.service_deadline=time.monotonic()+35
                self.create_timer(.01,self.pump)

            def fk(self,q):
                proc=bridge.kdl_process
                proc.stdin.write(' '.join(map(str,q))+'\n');proc.stdin.flush()
                row=proc.stdout.readline()
                if not row:raise RuntimeError('KDL worker closed')
                return json.loads(row)

            def status(self,msg):
                self.status_code=msg.data
                with bridge.lock:bridge.info['status']=msg.data

            def service(self,op):
                if self.future is not None:return
                client=self.service_clients[op]
                if client.service_is_ready():
                    self.future=client.call_async(Trigger.Request());self.operation=op
                    self.service_deadline=time.monotonic()+30

            def service_progress(self):
                if self.future is not None and self.future.done():
                    result=self.future.result()
                    if not result or not result.success:raise RuntimeError('Servo '+self.operation+' failed')
                    if self.operation=='start':self.started=True
                    if self.operation=='pause':
                        self.paused=True;self.pause_ns=bridge.clock();self.active_epoch=None;self.published=[]
                    if self.operation=='unpause':self.paused=False;self.active_epoch=self.pending_epoch
                    self.future=None;self.operation=None
                if time.monotonic()>self.service_deadline and (not self.started or self.future is not None):
                    raise TimeoutError('MoveIt service unavailable: '+str(self.operation))

            def zero(self):
                self.publish(None)

            def publish(self,command):
                intent=None if command is None else command['intent']
                stamp=self.get_clock().now().to_msg()
                twist=TwistStamped();twist.header.stamp=stamp
                twist.header.frame_id='controller_flange' if intent and intent['frame']=='tool' else 'controller_base'
                jog=JointJog();jog.header.stamp=stamp;jog.joint_names=self.names;jog.velocities=[0.]*6
                if intent:
                    if any(intent['joint_rad_s']):jog.velocities=intent['joint_rad_s']
                    else:
                        for i,k in enumerate('xyz'):
                            setattr(twist.twist.linear,k,intent['linear_m_s'][i])
                            setattr(twist.twist.angular,k,intent['angular_rad_s'][i])
                # Clear the unused channel as upstream prioritizes nonzero Twist.
                self.twist.publish(twist);self.jog.publish(jog)
                if command:
                    self.published.append(dict(sequence=command['sequence'],at_ns=bridge.clock(),
                        ros_stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec,epoch=command['epoch'],
                        kind='joint' if any(intent['joint_rad_s']) else 'twist',
                        intent=command['intent'],observation_received_ns=self.actual['received_ns']))
                    self.published=self.published[-8:]

            def pump(self):
                if bridge.process.poll() is not None:raise RuntimeError('MoveIt process exited')
                now=bridge.clock();sample=bridge.reader.snapshot();s=sample.get('status')
                fresh=bool(s and sample['connected'] and sample['age_ms'] is not None and 0<=sample['age_ms']<=100)
                self.actual=sample
                if fresh:
                    msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
                    msg.name=self.names;msg.position=np.deg2rad(s['joint_state']['q_actual']).tolist()
                    # QDActual is not treated as a verified velocity; preserve
                    # it in raw feedback. Servo's position path uses these q.
                    self.feedback.publish(msg)
                    if now-self.last_model_ns>=50_000_000:
                        self.model=self.fk(msg.position);self.last_model_ns=now
                        p=s['tool_vector_actual'];pose=[p[k] for k in ('x','y','z','rx','ry','rz')]
                        dp=float(np.linalg.norm(np.array(self.model['position_m'])*1000-pose[:3]))
                        dr=math.degrees((Rotation.from_quat(self.model['quaternion_xyzw']).inv()*
                            Rotation.from_euler('xyz',pose[3:],degrees=True)).magnitude())
                        self.model_error=dict(position_mm=dp,orientation_deg=dr)
                else:self.model_error=None
                aligned=bool(self.model_error and self.model_error['position_mm']<=1 and self.model_error['orientation_deg']<=1)
                self.service_progress()
                command,epoch=bridge.command_snapshot()
                active=bool(command and fresh and aligned and 0<=now-command['received_ns']<=250_000_000)
                if not self.started:
                    if fresh:self.service('start')
                elif not active or self.active_epoch!=epoch:
                    if not self.paused:
                        self.zero();self.service('pause')
                    elif active and self.future is None and now-self.pause_ns>=int(bridge.period*1e9):
                        # Paused cycles reset the upstream filter to fresh q.
                        self.pending_epoch=epoch;self.publish(command);self.service('unpause')
                elif self.future is None and now>=self.next_publish:
                    self.publish(command);self.next_publish=now+int(bridge.period*1e9)
                with bridge.lock:
                    bridge.info.update(ready=self.started and fresh and aligned,
                        phase='active' if active and self.active_epoch==epoch else 'paused' if self.started else 'starting',
                        error=None if aligned else 'Model/feedback not yet aligned',model_error=self.model_error,
                        urdf_sha256=profile['urdf_sha256'],session=str(runtime))

            def candidate(self,msg):
                now=bridge.clock();command,epoch=bridge.command_snapshot()
                if not command or self.active_epoch!=epoch or self.paused or self.future is not None:return
                if not 0<=now-command['received_ns']<=250_000_000 or not self.published:return
                if not self.actual or not 0<=now-self.actual['received_ns']<=100_000_000:return
                parts=msg.header.frame_id.split('/')
                if len(parts)!=2 or parts[0] not in ('e6_twist','e6_joint'):return
                matches=[p for p in self.published if str(p['ros_stamp_ns'])==parts[1]
                    and 'e6_'+p['kind']==parts[0] and p['epoch']==epoch]
                if len(matches)!=1:return
                source=matches[0]
                if not 0<=now-source['at_ns']<=100_000_000:return
                if len(msg.joint_names)!=6 or set(msg.joint_names)!=set(self.names) or len(msg.points)!=1:
                    raise ValueError('Invalid MoveIt joint trajectory')
                p=msg.points[0]
                if len(p.positions)!=6 or not all(math.isfinite(v) for v in p.positions):raise ValueError('Nonfinite MoveIt candidate')
                q=[p.positions[msg.joint_names.index(n)] for n in self.names]
                model=self.fk(q);self.counter+=1
                row=dict(id=self.counter,epoch=epoch,received_ns=now,joints_rad=q,
                    position_m=model['position_m'],quaternion_xyzw=model['quaternion_xyzw'],status=self.status_code,
                    upstream_trajectory_stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
                    input_association='selected input stamp from MoveIt metadata patch',
                    input_sequence=source['sequence'],source_input=source,
                    pre_action_received_ns=self.actual['received_ns'])
                with bridge.lock:
                    if epoch==bridge.epoch and bridge.command is not None:bridge.latest=row

        node=Adapter()
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        while not bridge.halt.is_set():executor.spin_once(timeout_sec=.01)
    finally:
        if executor:executor.shutdown(timeout_sec=1)
        if node:node.destroy_node()
        rclpy.shutdown(context=context)
        for proc in (bridge.kdl_process,bridge.process):
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2)
        log.close();kdl_log.close()
