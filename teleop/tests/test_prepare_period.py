"""Replay the recorded short first period without sending any device command."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from e6_observer.control_core import ControlCore
from e6_observer.control_math import controller_pose,pose
from e6_observer.kinematics_profile import load_profile
from test_hardware_control import Clock,setup,send,arm


@pytest.mark.parametrize('stage',['shadow','live'])
@pytest.mark.parametrize('partial_s',[.0005,.018122939,.049])
def test_prepare_partial_period_does_not_integrate_send_or_ignore_health(stage,partial_s):
    clock,device,core=setup(stage);arm(core,device)
    send(core,device,deadman=True,neutral=False,twist=[.002,0,0,0,0,0])
    clock.step(partial_s)
    before=core.target_p.copy()
    result=core.tick(device.state(),{'rgb':{'x5_receive_monotonic_ns':clock()}})
    assert core.armed and not core.faulted
    assert result['skipped_control_period_reason']=='first_period_after_prepare'
    assert result['candidate_action'] is result['effective_action'] is None
    assert np.array_equal(core.target_p,before)
    assert device.servo_count==device.grip_count==0
    # A new input replaces the old partial-period input; it isn't replayed.
    send(core,device,deadman=True,neutral=False,twist=[-.002,0,0,0,0,0])
    clock.step(.05)
    result=core.tick(device.state(),{'rgb':{'x5_receive_monotonic_ns':clock()}})
    assert result['candidate_action']['requested_tcp']['position_m'][0]<before[0]
    assert not core.faulted


@pytest.mark.parametrize('source',['rgb','robot'])
def test_prepare_wait_still_stops_on_stale_required_observation(source):
    clock,device,core=setup('live');arm(core,device);send(core,device,deadman=True)
    clock.step(.018)
    state=device.state();rgb=clock()
    if source=='robot':state['x5_receive_monotonic_ns']-=101_000_000
    else:rgb-=501_000_000
    result=core.tick(state,{'rgb':{'x5_receive_monotonic_ns':rgb}})
    assert not core.armed and core.faulted and result['effective_action'] is None
    assert device.servo_count==device.grip_count==0 and device.stop_count==1


def test_recorded_short_interval_bias_preserves_constraints_after_full_period():
    root=Path(__file__).resolve().parents[2]
    recorded=json.loads((root/'docs/evidence/teleop_resume_20261006/constraint_trigger_v50.json').read_text(encoding='utf-8'))
    clock=Clock()
    model,_=load_profile(root/'docs/evidence/teleop_local_integration_20261005/enabled_calibration_chord_v16/profile.json','192.168.1.4')
    class Replay:
        def __init__(self):self.ik_calls=0
        def ik(self,raw,seed,deadline):
            self.ik_calls+=1;q,_=model.solve(*pose(raw),np.radians(seed));return np.degrees(q).tolist()
        def fk(self,q,deadline):
            p,r=model.fk(np.radians(q));return controller_pose(p,r,recorded['pre_action_robot']['decoded']['controller_pose_raw_mm_deg'])
        def state(self):
            state=copy.deepcopy(recorded['pre_action_robot']);state['x5_receive_monotonic_ns']=clock();return state
        def servo(self,*args):raise AssertionError('Offline replay must never send ServoJ')
        def grip(self,*args):raise AssertionError('Offline replay must never send CAN')
    device=Replay();core=ControlCore(device,'shadow',clock=clock);core.connect()
    signal={k:recorded['input'][k] for k in ('twist','source','identity','mode','frame','fine')}
    send(core,device,**dict(signal,twist=[0]*6))
    core.command(dict(session=core.owner,operation='arm'),device.state(),clock())
    send(core,device,deadman=True,neutral=False,**signal)
    clock.step(recorded['dt_s'])
    partial=core.tick(device.state(),{'rgb':{'x5_receive_monotonic_ns':clock()}})
    assert partial['candidate_action'] is None and device.ik_calls==0 and not core.faulted
    clock.step(.05)
    result=core.tick(device.state(),{'rgb':{'x5_receive_monotonic_ns':clock()}})
    assert device.ik_calls==1 and not core.faulted
    assert result['ik_result']['accepted']
    assert max(abs(x) for x in result['ik_result']['joint_acceleration_rad_s2'])<=core.limits.joint_accel_rad_s2
    assert result['ik_result']['position_m']<=.001 and result['ik_result']['rotation_rad']<=np.radians(1)
