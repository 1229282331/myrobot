"""An arm release removes CAN capability without changing arm control."""
import json,threading
import pytest
from e6_observer.control_guardian import GripperLink,Commissioner
from e6_teleop.gripper_profile import gripper_profile
from test_hardware_control import setup


def test_disabled_profile_rejects_before_platform_or_can_setup():
    assert gripper_profile('disabled')['enabled'] is False
    with pytest.raises(ValueError,match='禁止配置CAN'):
        GripperLink('disabled')


def test_disabled_commissioner_never_calls_can_factory(monkeypatch):
    clock,device,core=setup('live');core.gripper_command_profile=gripper_profile('disabled')
    monkeypatch.setattr('e6_observer.control_guardian.time.perf_counter_ns',clock)
    created=[]
    commissioner=Commissioner(core,device,threading.Lock(),device.state,lambda:True,
        gripper_factory=lambda:created.append('CAN'))
    with pytest.raises(ValueError,match='禁止初始化'):
        commissioner.start('initialize-gripper')
    assert not created and not commissioner.gripper_attempted and not core.gripper_initialized


def test_arm_only_profile_cannot_select_active_gripper(tmp_path):
    from e6_teleop.acceptance_profile import verify_profile
    p=tmp_path/'profile.json'
    for scope,name in [('arm-only','empty64-medium'),('complete','disabled')]:
        p.write_text(json.dumps(dict(schema_version=1,complete=True,control_scope=scope,gripper_command_profile=name)))
        with pytest.raises(ValueError,match='disabled'):
            verify_profile(p)
