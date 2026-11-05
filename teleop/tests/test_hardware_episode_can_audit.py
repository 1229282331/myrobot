import copy
import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_hardware_episode import audit_gripper_action
from omnipicker.tools.verify_can_control import FRAME


def sample():
    clock=2_000_000_000
    command=bytes((0,51,25,64,25,25,0,0));data=bytes((0,0,51,0,20,25,0,0))
    feedback=dict(raw_hex=data.hex(),fault=0,state=0,state_name='到位',position_raw=51,
        position_percent=20,velocity_raw=0,force_raw=20,echo_hex=data[5:].hex(),
        command_position_raw=51,host_perf_counter_ns=clock+4_000_000,round_trip_ms=1)
    # Use the pinned parser's state spelling; physical semantics are unchanged.
    from omnipicker.tools.verify_can_control import STATE_NAMES
    feedback['state_name']=STATE_NAMES[0]
    action=dict(gripper=.2,gripper_acceptance='request_associated_feedback',gripper_feedback=feedback,
        gateway_accepted_ns=clock+2_000_000,transaction_started_ns=clock+10_000)
    row=dict(observation={'gripper':.2},sources={'gripper':dict(valid=True,x5_receive_monotonic_ns=clock-50_000_000,raw_hex=data.hex(),position_raw=51)},
        control={'can_events':[dict(kind='tx',host_perf_counter_ns=clock+3_000_000,send_confirmed=True,can_id=1,data_hex=command.hex()),
        dict(kind='rx',host_perf_counter_ns=clock+4_000_000,frame_hex=FRAME.pack(1,8,data).hex(),flags=0)]})
    return row,action,clock


def test_raw_can_pair_and_pre_action_observation_pass():
    assert audit_gripper_action(*sample())


@pytest.mark.parametrize('violation',['next-state','target-filled-state','target-label','echo','unrelated-id','missing-send','late-can'])
def test_invalid_or_uncertain_can_cannot_be_an_accepted_action(violation):
    row,action,clock=sample()
    if violation=='next-state':row['sources']['gripper']['x5_receive_monotonic_ns']=clock+1
    if violation=='target-filled-state':row['observation']['gripper']=.3
    if violation=='target-label':action['gripper']=.3
    if violation=='echo':row['control']['can_events'][1]['flags']=0x800
    if violation=='unrelated-id':row['control']['can_events'][1]['frame_hex']=FRAME.pack(2,8,bytes.fromhex(action['gripper_feedback']['raw_hex'])).hex()
    if violation=='missing-send':row['control']['can_events']=row['control']['can_events'][1:]
    if violation=='late-can':action['transaction_started_ns']=clock-50_000_000
    with pytest.raises(ValueError):audit_gripper_action(row,action,clock)


def test_combined_can_transaction_uses_explicit_budget_and_preserves_legacy_default():
    row,action,clock=sample();action['transaction_started_ns']=clock-56_000_000
    row['control']['tick']={'status':{'limits':{'transaction_s':.075}}}
    assert audit_gripper_action(row,action,clock)
    row['control'].pop('tick')
    with pytest.raises(ValueError):audit_gripper_action(row,action,clock)
    row['control']['tick']={'status':{'limits':{'transaction_s':.075}}}
    action['transaction_started_ns']=clock-72_000_000
    with pytest.raises(ValueError):audit_gripper_action(row,action,clock)
