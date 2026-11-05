"""Minimal official ServoP qualification, separate from webpage acceptance.

Defaults to read-only queries. Never enables, clears errors, alters parameters,
or sends CAN. Explicit zero qualification uses the freshly read Tool0 pose.
"""
import argparse
import base64
from dataclasses import asdict
import json
from pathlib import Path
import queue
import re
import time

import numpy as np
from scipy.spatial.transform import Rotation
from dobot_sdk.api.motion import Motion
from dobot_sdk.api.robot_control import RobotControl
from .dobot_transport import FramedDobotConnection
from .feedback import FeedbackReader


def checked(reader, generation=None):
    sample = reader.snapshot()
    status = sample['status']
    if status is None or not sample['connected'] or sample['age_ms'] is None or not 0<=sample['age_ms']<=100:
        raise RuntimeError('Actual feedback unavailable or older than 100 ms')
    if generation is not None and sample['generation'] != generation:
        raise RuntimeError('Feedback connection restarted')
    if not status['enable_status'] or status['error_status'] or status['collision_state']:
        raise RuntimeError('Device is not enabled and healthy')
    if status['robot_mode'] not in (5, 7):
        raise RuntimeError('Device mode not available for servo qualification')
    if status['user_coordinate'] != 0 or status['tool_coordinate'] != 0:
        raise RuntimeError('Probe requires current User0 and Tool0; does not change them')
    if np.max(np.abs(status['user_value'] + status['tool_value'])) > 1e-6:
        raise RuntimeError('Probe requires zero User0/Tool0 offsets')
    return sample


def success(reply):
    if int(reply.split(',', 1)[0]) != 0:
        raise RuntimeError('Controller rejected command: ' + reply)
    return reply


def checked_camera(path):
    if path is None:raise RuntimeError('Stop qualification requires active independent RGB observation')
    health=json.loads(path.read_text())
    age=(time.perf_counter_ns()-health['rgb_received_ns'])/1e6
    if health.get('serial')!='AY4A1520111' or not 0<=age<=500 or health.get('error') or health.get('closed_ns'):
        raise RuntimeError('Independent RGB is not fresh')


def reply_vector(reply):
    success(reply)
    match=re.match(r'^0,\{([^{}]*)\},',reply)
    if not match:raise ValueError('Invalid official kinematics reply')
    values=np.array([float(x) for x in match[1].split(',')])
    if values.shape!=(6,) or not np.isfinite(values).all():raise ValueError('Invalid kinematics values')
    return values


def joint_z_plan(robot,pose,base_joints,check,axis=2):
    """Diagnostic only: native IK/FK for a NEW +1.6 mm line, not a teleop solver."""
    if axis not in (0,2):raise ValueError('Only the explicit X/Z qualification axes are available')
    result=[];previous=np.array(base_joints)
    for i in range(1,21):
        check()
        target=np.array(pose);target[axis]+=i*.08
        joints=reply_vector(robot.InverseKin(target,use_joint_near=1,joint_near=base_joints,user=0,tool=0))
        fk=reply_vector(robot.PositiveKin(joints,user=0,tool=0))
        linear=float(np.linalg.norm(target[:3]-fk[:3]))
        angular=float(np.rad2deg((Rotation.from_euler('xyz',target[3:],degrees=True)*
                                 Rotation.from_euler('xyz',fk[3:],degrees=True).inv()).magnitude()))
        if linear>1 or angular>1 or np.max(abs(joints-base_joints))>2:
            raise RuntimeError('Native IK/FK line qualification rejected')
        if np.max(abs(joints-previous))>.1:
            raise RuntimeError('Native line exceeds 2 degree/s joint step')
        result.append(dict(pose_mm_deg=target.tolist(),joints_deg=joints.tolist(),
                           fk_mm_deg=fk.tolist(),linear_residual_mm=linear,angular_residual_deg=angular))
        previous=joints
    return result


def micro_rotation_plan(pose, *, axis=1, speed_deg_s=-.25, period_s=.05, count=40):
    """Bounded base-frame diagnostic, independent of application integration."""
    if axis not in (0,1) or abs(speed_deg_s) != .25 or period_s != .05 or count != 40:
        raise ValueError('Micro rotation probe is fixed to 0.25 degree/s for two seconds')
    initial=Rotation.from_euler('xyz',pose[3:],degrees=True)
    vectors=np.zeros((count,3));vectors[:,axis]=np.arange(1,count+1)*period_s*speed_deg_s
    rotations=Rotation.from_rotvec(vectors,degrees=True)*initial
    return np.c_[np.tile(pose[:3],(count,1)),rotations.as_euler('xyz',degrees=True)].tolist()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['query', 'zero', 'zero-joints', 'line-z', 'stop-z', 'stop-j4','stop-z-joints','stop-x-joints','stop-tool-rx','micro-base-ry'], default='query')
    p.add_argument('--camera-health',type=Path)
    p.add_argument('--servo-gain',type=int,choices=[500,750],default=500)
    p.add_argument('--servo-duration-s',type=float,choices=[.05,.1],default=.05,
                   help='SDK point run time t; probe cadence and trajectory remain 50 ms')
    p.add_argument('--rotation-speed-deg-s',type=float,choices=[.5,1,2.5],default=1,
                   help='Only stop-tool-rx: bounded half-second native diagnostic, not page acceptance')
    p.add_argument('--host', default='192.168.1.4')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    reader = FeedbackReader(a.host)
    sub = reader.subscribe(4096)
    conn = FramedDobotConnection(a.host)
    robot, motion = RobotControl(conn), Motion(conn)
    summary = dict(mode=a.mode, qualification_only=True, webpage_acceptance=False,
                   error=None, stop_error=None, motion_attempted=False, can_sends=0,
                   enable_commands=0, parameter_changes=0,servo_gain=a.servo_gain,
                   servo_period_s=.05,servo_duration_s=a.servo_duration_s)
    reader.start()
    try:
        deadline = time.monotonic() + 3
        while reader.snapshot()['status'] is None and time.monotonic() < deadline:
            time.sleep(.02)
        start = checked(reader)
        generation = start['generation']
        conn.connect()
        summary['queries'] = [success(robot.RobotMode()), success(robot.GetPose()),
                              success(robot.GetAngle()), success(robot.GetErrorID())]
        if a.mode!='query':
            stable = []
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                sample = checked(reader, generation)
                checked_camera(a.camera_health)
                if sample['status']['robot_mode'] != 5 or sample['status']['run_queued_cmd']:
                    raise RuntimeError('Qualification must start stationary with empty queue')
                stable.append(sample['status']['joint_state']['q_actual'])
                time.sleep(.02)
            if np.max(np.ptp(stable, axis=0)) > .05:
                raise RuntimeError('Initial five seconds were not stable')
            summary['before'] = before = checked(reader, generation)
            pose = [before['status']['tool_vector_actual'][key] for key in ('x','y','z','rx','ry','rz')]
            base_joints = np.array(before['status']['joint_state']['q_actual'])
            # ACK pacing only: never wait for idle between streaming targets.
            offsets = [0.] * 20 if a.mode in ('zero','zero-joints') else (
                [i / 20 for i in range(1, 21)] + [1.] * 5 +
                [i / 20 for i in range(19, -1, -1)] + [0.] * 5)
            if a.mode in ('stop-z','stop-z-joints','stop-x-joints'):
                # Same 1.6 mm/s, 50 ms cadence as the formal-page failure;
                # stop directly from streaming, without a pre-stop hold.
                offsets=[i*.08 for i in range(1,21)]
                summary['trajectory']=dict(axis='base_x' if a.mode=='stop-x-joints' else 'base_z',speed_mm_s=1.6,distance_mm=1.6,
                    period_s=.05,servo_duration_s=a.servo_duration_s,aheadtime=50,gain=a.servo_gain,return_motion=False)
            joint_plan=None
            micro_plan=None
            if a.mode=='micro-base-ry':
                micro_plan=micro_rotation_plan(pose)
                offsets=list(range(len(micro_plan)))
                summary['trajectory']=dict(axis='base_ry',speed_deg_s=-.25,distance_deg=-.5,
                    period_s=.05,servo_duration_s=a.servo_duration_s,aheadtime=50,gain=a.servo_gain,
                    stop_phase='after_final_reply_without_extra_sleep',return_motion=False)
            if a.mode in ('stop-z-joints','stop-x-joints'):
                def check_plan():
                    latest=checked(reader,generation);checked_camera(a.camera_health)
                    if np.max(abs(np.array(latest['status']['joint_state']['q_actual'])-base_joints))>.05:
                        raise RuntimeError('Actual state changed during read-only planning')
                joint_plan=joint_z_plan(robot,pose,base_joints,check_plan,axis=0 if a.mode=='stop-x-joints' else 2)
                summary['native_ik_fk_plan']=joint_plan
                check_plan()
            if a.mode=='stop-j4':
                # Match the order of magnitude of the failing ServoP J4
                # trajectory, without Cartesian IK or a reused start pose.
                offsets=[i*.016 for i in range(1,21)]
                summary['trajectory']=dict(axis='joint4',speed_deg_s=.32,distance_deg=.32,
                    period_s=.05,servo_duration_s=a.servo_duration_s,aheadtime=50,gain=a.servo_gain,return_motion=False)
            if a.mode=='stop-tool-rx':
                # Post-multiplying by tool Rx is exactly incrementing the
                # controller's XYZ-Euler roll at fixed pitch/yaw. Keep this
                # official SDK diagnostic independent of app trajectory code.
                offsets=[i*.05*a.rotation_speed_deg_s for i in range(1,11)]
                summary['trajectory']=dict(axis='tool_rx',speed_deg_s=a.rotation_speed_deg_s,
                    distance_deg=.5*a.rotation_speed_deg_s,period_s=.05,
                    servo_duration_s=a.servo_duration_s,aheadtime=50,gain=a.servo_gain,
                    stop_phase='after_final_reply_without_extra_sleep',return_motion=False)
            for index,offset in enumerate(offsets):
                started = time.monotonic()
                latest = checked(reader, generation)
                checked_camera(a.camera_health)
                excursion_limit = .25 if a.mode in ('zero','zero-joints') else 2.
                if np.max(np.abs(np.array(latest['status']['joint_state']['q_actual']) - base_joints)) > excursion_limit:
                    raise RuntimeError('Qualification exceeded joint excursion limit')
                summary['motion_attempted'] = True
                if a.mode in ('zero-joints','stop-j4','stop-z-joints','stop-x-joints'):
                    if joint_plan is not None:target=joint_plan[index]['joints_deg']
                    else:
                        target=base_joints.copy();target[3]+=offset
                    success(motion.ServoJ(target,t=a.servo_duration_s,aheadtime=50,gain=a.servo_gain))
                else:
                    if micro_plan is not None:target=micro_plan[index]
                    else:
                        target=pose.copy();target[3 if a.mode=='stop-tool-rx' else 2]+=offset
                    success(motion.ServoP(target, t=a.servo_duration_s, aheadtime=50, gain=a.servo_gain))
                if a.mode not in ('stop-tool-rx','micro-base-ry') or index<len(offsets)-1:
                    time.sleep(max(0, .05 - (time.monotonic() - started)))
    except Exception as exc:
        summary['error'] = str(exc)
    finally:
        if summary['motion_attempted']:
            summary['release_received_ns'] = time.perf_counter_ns()
            try:
                conn.set_timeout(receive_timeout=1.5)
                summary['stop_reply'] = success(robot.Stop())
                summary['stop_ack_ns'] = time.perf_counter_ns()
            except Exception as exc:
                summary['stop_error'] = str(exc)
            window = []
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                snap = reader.snapshot()
                status = snap['status']
                if status and snap['connected'] and snap['age_ms'] <= 100:
                    window.append((snap['received_ns'], status['joint_state']['q_actual']))
                    window = [row for row in window if snap['received_ns']-row[0] <= 250_000_000]
                    if (len(window) >= 10 and window[-1][0]-window[0][0] >= 200_000_000
                        and status['robot_mode'] == 5 and not status['run_queued_cmd']
                        and np.max(np.ptp([row[1] for row in window], axis=0)) < .02):
                        # Retain first window detection, not the last sample.
                        # Formal physical stop metrics still use raw feedback.
                        summary.setdefault('stationary_ns', time.perf_counter_ns())
                        if a.mode not in ('stop-z','stop-j4','stop-z-joints','stop-x-joints','stop-tool-rx','micro-base-ry'):break
                time.sleep(.01)
        summary['after'] = reader.snapshot()
        conn.disconnect()
        reader.close()
        summary['commands'] = list(conn.events)
        summary['feedback_drops'] = sub.dropped
        summary['feedback_events'] = list(reader.events)
        count = 0
        with (a.output/'frames.jsonl').open('x', encoding='utf-8') as f:
            while True:
                try:
                    generation, sequence, frame = sub.queue.get_nowait()
                except queue.Empty:
                    break
                count += 1
                f.write(json.dumps(dict(generation=generation, sequence=sequence,
                    received_ns=frame.received_ns, status=asdict(frame.status),
                    wire_base64=base64.b64encode(frame.prefix+frame.raw).decode()))+'\n')
        summary['frames'] = count
        (a.output/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('before','after','commands','native_ik_fk_plan')}))
    return int(bool(summary['error'] or summary['stop_error'] or summary['feedback_drops']))


if __name__ == '__main__':
    raise SystemExit(main())
