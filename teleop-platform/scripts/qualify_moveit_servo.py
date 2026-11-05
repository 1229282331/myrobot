"""Run the unmodified MoveIt Servo node with a virtual E6 plant, NEVER hardware.

No Dobot imports, dashboard sockets, feedback subscriptions or CAN paths.
The official model is currently unaligned; this tests library integration only.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_DOMAIN_ID']='87'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
import yaml


def parameters(source, servo_config):
    urdf=(source/'dobot_rviz/urdf/me6_robot.urdf').read_text()
    urdf=urdf.replace('package://dobot_rviz/',(source/'dobot_rviz').as_uri()+'/')
    tree=ET.fromstring(urdf)
    for mesh in tree.findall('./link/collision/geometry/mesh'):
        path=source/'collision'/Path(mesh.get('filename')).name
        if not path.is_file():raise ValueError('Missing validated collision hull: '+str(path))
        mesh.set('filename',path.as_uri())
    urdf=ET.tostring(tree,encoding='unicode')
    srdf=(source/'me6_moveit/config/me6_robot.srdf').read_text()
    params=yaml.safe_load(servo_config.read_text())
    params.update(command_in_type='speed_units',publish_period=.05,move_group_name='me6_group',
        planning_frame='base_link',ee_frame_name='Link6',robot_link_command_frame='base_link',
        joint_topic='/e6_qualification/joint_states',command_out_topic='/e6_qualification/candidates',
        cartesian_command_in_topic='/e6_qualification/twist',joint_command_in_topic='/e6_qualification/jog',
        incoming_command_timeout=.25,check_collisions=True)
    limits={f'joint{i}':dict(has_velocity_limits=True,max_velocity=math.radians(2),
            has_acceleration_limits=True,max_acceleration=math.radians(4)) for i in range(1,7)}
    return {'robot_description':urdf,'robot_description_semantic':srdf,
        'robot_description_planning':{'joint_limits':limits},'moveit_servo':params}


class VirtualPlant(Node):
    def __init__(self,source,file,seed=None):
        super().__init__('e6_servo_virtual_plant')
        srdf=ET.parse(source/'me6_moveit/config/me6_robot.srdf').getroot()
        home=srdf.find("group_state[@name='home']")
        self.names=[f'joint{i}' for i in range(1,7)]
        self.q=[float(home.find(f"joint[@name='{name}']").get('value')) for name in self.names]
        if seed is not None:
            if len(seed)!=6 or not all(math.isfinite(x) for x in seed):raise ValueError('Invalid virtual seed')
            self.q=[math.radians(x) for x in seed]
        self.out=file;self.case='startup';self.rows=[];self.status=None
        self.feedback=self.create_publisher(JointState,'/e6_qualification/joint_states',10)
        self.twist=self.create_publisher(TwistStamped,'/e6_qualification/twist',10)
        self.jog=self.create_publisher(JointJog,'/e6_qualification/jog',10)
        self.create_subscription(JointTrajectory,'/e6_qualification/candidates',self.candidate,10)
        self.create_subscription(Int8,'/servo_node/status',lambda m:setattr(self,'status',m.data),10)
        self.create_timer(.01,self.publish_state)

    def publish_state(self):
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=self.names;msg.position=self.q;msg.velocity=[0.]*6;self.feedback.publish(msg)

    def candidate(self,msg):
        if set(msg.joint_names)!=set(self.names) or not msg.points:raise ValueError('Invalid candidate joint names')
        point=msg.points[0]
        if len(point.positions)!=6 or not all(math.isfinite(v) for v in point.positions):
            raise ValueError('Invalid MoveIt candidate positions')
        self.q=[point.positions[msg.joint_names.index(n)] for n in self.names]
        row=dict(received_ns=time.perf_counter_ns(),case=self.case,positions_rad=self.q,
                 velocities_rad_s=list(point.velocities),status=self.status)
        self.rows.append(row);self.out.write(json.dumps(row)+'\n');self.out.flush()

    def pump(self,seconds,send=None):
        deadline=time.monotonic()+seconds;next_send=time.monotonic()
        while time.monotonic()<deadline:
            if send and time.monotonic()>=next_send:send();next_send=time.monotonic()+.05
            rclpy.spin_once(self,timeout_sec=.003)

    def service(self,name):
        cli=self.create_client(Trigger,'/servo_node/'+name)
        try:
            deadline=time.monotonic()+15
            while not cli.service_is_ready() and time.monotonic()<deadline:self.pump(.1)
            if not cli.service_is_ready():raise TimeoutError(name+' service unavailable')
            # Service discovery can precede completion of mesh/model loading.
            # This is startup-only, unrelated to any real control deadline.
            future=cli.call_async(Trigger.Request());deadline=time.monotonic()+(30 if name=='start_servo' else 5)
            while not future.done() and time.monotonic()<deadline:self.pump(.01)
            if not future.done() or not future.result().success:raise RuntimeError(name+' failed')
        finally:self.destroy_client(cli)

    def send(self,kind,axis,value):
        if kind=='cartesian':
            msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
            setattr(msg.twist.linear if axis<3 else msg.twist.angular,'xyz'[axis%3],value)
            self.twist.publish(msg)
        else:
            msg=JointJog();msg.header.stamp=self.get_clock().now().to_msg()
            msg.joint_names=[self.names[axis]];msg.velocities=[value];self.jog.publish(msg)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--servo-config',type=Path,required=True)
    p.add_argument('--seed-degrees',type=Path,help='Six virtual joint values, NOT a robot command')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    config=a.output/'params.yaml';config.write_text(yaml.safe_dump({'/servo_node':{'ros__parameters':parameters(a.source,a.servo_config)}}))
    result=dict(hardware_connected=False,motion_commands=0,can_sends=0,model_aligned=False,
        production_eligible=False,library='MoveIt Servo 2.5.10',cases=[],error=None)
    seed=json.loads(a.seed_degrees.read_text()) if a.seed_degrees else None
    result['virtual_seed_degrees']=seed
    rclpy.init();process=None;node=None
    try:
        with (a.output/'servo.log').open('w') as log,(a.output/'candidates.jsonl').open('w') as candidates:
            process=subprocess.Popen(['/opt/ros/humble/lib/moveit_servo/servo_node_main','--ros-args',
                '--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            node=VirtualPlant(a.source,candidates,seed);node.pump(2);node.service('start_servo')
            for kind in ('joint','cartesian'):
                for axis in range(6):
                    for sign in (-1,1):
                        name=f'{kind}_{axis}_{sign}';node.case=name;seed=list(node.q);count=len(node.rows)
                        speed=.001 if kind=='cartesian' and axis<3 else math.radians(.5)
                        node.pump(.75,lambda:node.send(kind,axis,sign*speed))
                        node.case=name+'_release';node.pump(.5,lambda:node.send(kind,axis,0.))
                        delta=[x-y for x,y in zip(node.q,seed)]
                        result['cases'].append(dict(name=name,candidates=len(node.rows)-count,
                            delta_q_rad=delta,status=node.status,generated_motion=max(map(abs,delta))>1e-7,
                            statuses=sorted(set(str(r['status']) for r in node.rows[count:]))))
            node.service('pause_servo');node.pump(.5);node.service('unpause_servo')
            node.service('stop_servo');result['pause_resume_services']=True
            result['candidate_count']=len(node.rows)
            result['all_cases_generated_motion']=all(c['generated_motion'] for c in result['cases'])
    except Exception as exc:result['error']=str(exc)
    finally:
        if process:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    result['forced_exit']=True;os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        result['killed_owned_virtual_process']=True
                        os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
            result['servo_exit_code']=process.returncode
        if node:node.destroy_node()
        rclpy.shutdown()
        (a.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result))
    return int(result['error'] is not None)


if __name__=='__main__':raise SystemExit(main())
