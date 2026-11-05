import importlib.util
from pathlib import Path

import pytest

spec=importlib.util.spec_from_file_location('gripper_probe',Path(__file__).parents[1]/'scripts/qualify_gripper_updates.py')
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


def test_fixed_and_ramp_use_identical_endpoints_without_overshoot():
    fixed=list(probe.comparison('fixed'));ramp=list(probe.comparison('ramp'))
    assert fixed[0]==ramp[0] and fixed[-2:]==ramp[-2:]
    positions=[target for label,target,_ in ramp if label=='ramp_close']
    assert positions[-1]==70/255
    assert all(0 < a-b <= .025*.05+1e-12 for a,b in zip([.35]+positions,positions))
    assert all(70/255<=target<=.35 for _,target,_ in ramp)


def test_batched_diagnostic_preserves_explicit_four_count_updates():
    stages=list(probe.comparison('batch-four'))
    positions=[round(target*255) for label,target,_ in stages if label=='batch_close']
    assert positions==[85,81,77,73,69]
    assert stages[-2][1]==69/255 and stages[-1][1]==.35


def test_probe_requires_enabled_idle_arm_and_fresh_actual_camera():
    sample=dict(connected=True,age_ms=1,status=dict(enable_status=True,error_status=False,
                collision_state=0,robot_mode=5,run_queued_cmd=0))
    camera=dict(serial='AY4A1520111',rgb_received_ns=1)
    probe.check_health(sample,camera,10)
    for key,value in [('enable_status',False),('error_status',True),('collision_state',1),('robot_mode',7),('run_queued_cmd',1)]:
        with pytest.raises(RuntimeError):probe.check_health({**sample,'status':{**sample['status'],key:value}},camera,10)
    with pytest.raises(RuntimeError):probe.check_health({**sample,'age_ms':101},camera,10)
    with pytest.raises(RuntimeError):probe.check_health(sample,camera,500_000_002)
    with pytest.raises(RuntimeError):probe.check_health(sample,{**camera,'serial':'wrong'},10)
