"""E6 configurations for the pinned MoveIt Servo distributions."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .joint_range import JOINT_MARGIN_DEG


def load_model(path):
    path=Path(path)
    profile=json.loads((path/'model/profile.json').read_text())
    for name,key in [('e6.urdf','urdf_sha256'),('e6.srdf','srdf_sha256')]:
        if hashlib.sha256((path/'model'/name).read_bytes()).hexdigest()!=profile[key]:
            raise ValueError('Model artifact differs from its reviewed profile')
    tree=ET.parse(path/'model/e6.urdf')
    for mesh in tree.findall('./link/visual/geometry/mesh'):
        file=path/'meshes'/Path(mesh.get('filename')).name
        if not file.is_file():raise ValueError('Missing visual mesh: '+str(file))
        mesh.set('filename',file.as_uri())
    for mesh in tree.findall('./link/collision/geometry/mesh'):
        file=path/'collision'/Path(mesh.get('filename')).name
        if not file.is_file():raise ValueError('Missing conservative collision hull: '+str(file))
        mesh.set('filename',file.as_uri())
    return {'robot_description':ET.tostring(tree.getroot(),encoding='unicode'),
        'robot_description_semantic':(path/'model/e6.srdf').read_text()},profile


def load_bundle(path, period=.05):
    import yaml
    path=Path(path);model,profile=load_model(path)
    params=yaml.safe_load((path/'servo_config.yaml').read_text())
    params.update(command_in_type='speed_units',publish_period=period,move_group_name='me6_group',
        planning_frame='controller_base',ee_frame_name='controller_flange',robot_link_command_frame='controller_base',
        joint_topic='/e6_adapter/joint_states',command_out_topic='/e6_adapter/candidates',
        cartesian_command_in_topic='/e6_adapter/twist',joint_command_in_topic='/e6_adapter/jog',
        incoming_command_timeout=.25,check_collisions=True)
    limits={f'joint{i}':dict(has_velocity_limits=True,max_velocity=math.radians(2),
        has_acceleration_limits=True,max_acceleration=math.radians(4)) for i in range(1,7)}
    return dict(model,robot_description_planning={'joint_limits':limits},moveit_servo=params),profile


def load_jazzy_bundle(path, device_period=.05, motion_profile='commissioning', tcp_profile='flange', collision_attachments=False):
    import yaml
    from .motion_profiles import get_profile
    speed=get_profile(motion_profile)
    if device_period!=.05:raise ValueError('Only the qualified 20 Hz device cadence is configured')
    path=Path(path);model,profile=load_model(path)
    from .tool_frame import get_tool
    tool=get_tool(tcp_profile)
    model=tool.attach(model)
    from .assembly_model import attach
    assembly_id=None
    if collision_attachments:model,assembly_id=attach(model,tcp_profile)
    params=yaml.safe_load((path/'servo_jazzy.yaml').read_text())
    params.update(publish_period=.01,max_expected_latency=.1,move_group_name='me6_group',
        joint_topic='/e6_adapter/joint_states',joint_command_in_topic='/e6_adapter/jog',
        cartesian_command_in_topic='/e6_adapter/twist',command_out_topic='/e6_adapter/candidates',
        status_topic='/e6_adapter/status',command_in_type='speed_units',incoming_command_timeout=.25,
        check_collisions=True,apply_twist_commands_about_ee_frame=True,
        lower_singularity_threshold=speed.singularity_slowdown,
        hard_stop_singularity_threshold=speed.singularity_stop,
        joint_limit_margins=[math.radians(JOINT_MARGIN_DEG)],smoothing_filter_plugin_name='e6_reviewed_smoothing::AccelerationLimitedPlugin')
    limits={f'joint{i}':speed.joint_limits() for i in range(1,7)}
    extra={}
    if collision_attachments and assembly_id:
        from .gripper_geometry import geometry
        grip=geometry()
        if grip:extra=dict(e6_gripper_joint_names=grip['joint_names'],e6_gripper_links=grip['movable_links'],
                           e6_gripper_poses=[float(v) for row in grip['joints_rad_by_raw'] for v in row],
                           e6_gripper_fallback=grip['fallback_link'])
    return dict(model,robot_description_planning={'joint_limits':limits},moveit_servo=params,**extra,
                robot_description_kinematics={'me6_group':{
                    'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution':.005,'kinematics_solver_timeout':.005,
                    'epsilon':1e-9}},
                e6_control_frame=tool.frame,
                update_period=.01,planning_group_name='me6_group'),dict(profile,tcp=tool.description(),assembly_id=assembly_id)
