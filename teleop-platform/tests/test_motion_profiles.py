import math
from types import SimpleNamespace
import pytest
from e6_phosphobot.motion_profiles import get_profile
from e6_phosphobot.gateway import NativeExecutor
from e6_phosphobot.moveit_config import load_jazzy_bundle


def test_default_preserves_commissioning_limits():
    p=get_profile()
    assert p.input_limits().linear_m_s==.002
    assert p.input_limits().angular_rad_s==math.radians(1)
    assert p.joint_limits()['max_velocity']==math.radians(2)
    assert p.joint_limits()['max_acceleration']==math.radians(4)


def test_goal_profile_flows_into_capabilities_and_solver(monkeypatch,tmp_path):
    p=get_profile('normal')
    (tmp_path/'servo_jazzy.yaml').write_text('{}')
    monkeypatch.setattr('e6_phosphobot.moveit_config.load_model',lambda path: ({},{}))
    config,_=load_jazzy_bundle(tmp_path,motion_profile='normal')
    assert all(q==p.joint_limits() for q in config['robot_description_planning']['joint_limits'].values())
    assert config['moveit_servo']['lower_singularity_threshold']==p.singularity_slowdown
    assert config['moveit_servo']['hard_stop_singularity_threshold']==p.singularity_stop
    assert config['moveit_servo']['smoothing_filter_plugin_name']=='e6_reviewed_smoothing::AccelerationLimitedPlugin'
    executor=NativeExecutor('unused','shadow',None,moveit_bundle=tmp_path,motion_profile='normal')
    limits=executor.limits
    assert limits.linear_m_s==.03 and limits.angular_rad_s==math.radians(10)
    assert limits.joint_rad_s==math.radians(10) and limits.gripper_rate_s==.1


def test_normal_soft_start_retains_speed_but_limits_acceleration_change():
    p=get_profile('normal')
    assert p.input_ramp_time_constant_s==.2
    assert p.joint_limits()['max_acceleration']==math.radians(30)
    assert p.joint_limits()['max_velocity']==math.radians(30)
    assert p.description()['input_ramp_time_constant_s']==.2
    for name in ('commissioning','intermediate'):
        assert get_profile(name).input_ramp_time_constant_s==0


def test_unrecognised_profile_is_rejected_before_opening_devices():
    with pytest.raises(ValueError):NativeExecutor('unused','live',None,motion_profile='fastest')
    with pytest.raises(ValueError):NativeExecutor('unused','live',None,motion_profile='normal')
