from copy import deepcopy
from pathlib import Path
import importlib.util
import json
import math
import numpy as np
import pytest
from e6_phosphobot.intent import Intent
from e6_phosphobot.moveit_bridge import MoveItBridge
from test_gateway import setup


def intent(**kw):
    return Intent(**(dict(linear_m_s=[.002,0,0],angular_rad_s=[0]*3,joint_rad_s=[0]*6,
                         gripper_rate_s=0,deadman=True,frame='base')|kw))


def test_release_discards_every_prior_epoch_candidate_and_never_replays():
    now=[100_000_000];b=MoveItBridge('.',None,clock=lambda:now[0])
    b.submit(intent(),7);epoch=b.epoch
    b.latest=dict(id=1,epoch=epoch,received_ns=now[0])
    assert b.take()['id']==1 and b.take() is None
    b.stop();b.submit(intent(),8)
    b.latest=dict(id=2,epoch=epoch,received_ns=now[0])
    assert b.take() is None
    b.latest=dict(id=3,epoch=b.epoch,received_ns=now[0]-100_000_001)
    assert b.take() is None
    b.latest=dict(id=4,epoch=b.epoch,received_ns=now[0])
    assert b.take()['id']==4
    b.latest=dict(id=5,epoch=b.epoch,received_ns=now[0],valid_until_ns=now[0]-1)
    assert b.take() is None
    report=b.snapshot()
    assert report['take_counts']==dict(accepted=2,no_candidate=1,inactive_epoch=1,candidate_age=1,sample_deadline=1)
    assert report['last_take']['reason']=='sample_deadline'
    assert report['last_take']['deadline_remaining_ms']<0
    # Diagnostics do not extend validity or replay the discarded candidate.
    assert b.take() is None
    assert b.snapshot()['last_take']['reason']=='already_consumed'


class Bridge:
    def __init__(self):self.info=dict(ready=True,error=None);self.row=None;self.calls=[]
    def snapshot(self):return self.info
    def submit(self,i,s):self.calls.append((i,s))
    def stop(self):self.row=None;self.calls.append('stop')
    def take(self):row=self.row;self.row=None;return row


def row(now,seq=2):
    return dict(input_sequence=seq,received_ns=now,joints_rad=[0,0,0,0,0,.0001],
                position_m=[0,0,.4],quaternion_xyzw=[0,0,0,1],status=0)


def test_fresh_input_primes_before_tick_but_only_tick_writes_device():
    e,d,send,step,now=setup();e.moveit=Bridge()
    tick=e.last_tick_ns
    send(deadman=True,linear_m_s=[.002,0,0])
    assert e.moveit.calls[-1][1]==e.session.sequence
    assert e.last_tick_ns==tick and not d.calls
    e.moveit.row=row(now[0],e.session.sequence)
    step()
    assert [c[0] for c in d.calls]==['ServoJ']
    # Subsequent inputs only replace computational intent, never send early.
    for _ in range(4):send(deadman=True,linear_m_s=[-.002,0,0])
    assert [c[0] for c in d.calls]==['ServoJ']


def test_release_invalidates_primed_candidate_before_any_device_tick():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0])
    e.moveit.row=row(now[0],e.session.sequence)
    send()
    assert e.moveit.row is None and e.session.ready
    step()
    assert not d.calls


@pytest.mark.parametrize('stage',['shadow','live'])
@pytest.mark.parametrize('duration',[.05,.1])
def test_moveit_target_and_consumption_unchanged_by_sdk_duration(stage,duration):
    e,d,send,step,now=setup(stage,servo_duration_s=duration);e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0])
    candidate=row(now[0],e.session.sequence);e.moveit.row=candidate
    step()
    assert e.candidate['moveit']==candidate
    assert e.moveit.row is None
    if stage=='live':
        assert len(d.calls)==1 and d.calls[0][0]=='ServoJ'
        assert d.calls[0][2]==dict(t=duration,aheadtime=50,gain=500)
        np.testing.assert_allclose(d.calls[0][1][0],np.rad2deg(candidate['joints_rad']))
        send()
        assert [c[0] for c in d.calls]==['ServoJ','Stop']
    else:
        send();step()
        assert not d.calls


def test_only_initial_wake_enforces_minimum_send_gap_without_slowing_regular_ticks():
    from e6_phosphobot.gateway import wait_control_cycle
    class Event:
        def __init__(self,signaled=False):self.signaled=signaled;self.waits=[]
        def wait(self,timeout):self.waits.append(timeout);return self.signaled
        def clear(self):self.signaled=False
    # Tick starts at 50 ms, work ends at 60 ms. Wait 40 ms, not another 50.
    halt,wake=Event(),Event()
    wait_control_cycle(halt,wake,50_000_000,.05,60_000_000,clock=lambda:60_000_000)
    assert wake.waits==[.04] and not halt.waits
    # A new epoch can wake early, but rapid release/repress cannot burst sends.
    halt,wake=Event(),Event(True)
    wait_control_cycle(halt,wake,50_000_000,.05,40_000_000,clock=lambda:60_000_000)
    assert halt.waits==[.03] and not wake.signaled
    # Long idle first candidate has no extra delay; an overrun never catches up.
    halt,wake=Event(),Event(True)
    wait_control_cycle(halt,wake,50_000_000,.05,0,clock=lambda:60_000_000)
    assert halt.waits==[0]
    halt,wake=Event(),Event()
    wait_control_cycle(halt,wake,0,.05,0,clock=lambda:70_000_000)
    assert wake.waits==[0]


def test_cpp_producer_paced_availability_is_not_delayed_by_a_second_clock():
    e,d,send,step,now=setup();e.moveit=Bridge();e.moveit_api='cpp'
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0],e.session.sequence)
    step();assert len(d.calls)==1
    sent=e.last_moveit_sent_ns
    next_row=row(now[0],e.session.sequence);e.moveit.row=next_row
    now[0]=sent+49_000_000;e.tick()
    assert len(d.calls)==2 and e.moveit.row is None
    # The producer enforces its 50 ms step; variable transport time need not
    # make consecutive arrival times at least 50 ms apart. Release still wins.
    send()
    assert [c[0] for c in d.calls]==['ServoJ','ServoJ','Stop'] and e.moveit.row is None
    now[0]=sent+51_000_000;e.tick()
    assert [c[0] for c in d.calls]==['ServoJ','ServoJ','Stop']


def test_cpp_due_target_is_consumed_once_without_waiting_for_a_new_fixed_tick():
    e,d,send,step,now=setup();e.moveit=Bridge();e.moveit_api='cpp'
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0],e.session.sequence)
    step();sent=e.last_moveit_sent_ns
    now[0]=sent+50_000_000;e.tick()  # Candidate is not available at the tick.
    assert len(d.calls)==1
    now[0]=sent+58_000_000;e.moveit.row=row(now[0],e.session.sequence);e.tick()
    assert len(d.calls)==2 and e.last_moveit_sent_ns==now[0]
    now[0]+=1_000_000;e.tick()
    assert len(d.calls)==2


@pytest.mark.parametrize('case',['unprepared','neutral','gripper_only','model_failed',
    'stop_pending','stop_failed','feedback_stale','feedback_reconnected','alarm',
    'disabled','camera_stale','duplicate','stale_echo','source_change','out_of_range'])
def test_ineligible_input_never_primes_or_writes_device(case):
    e,d,send,step,now=setup();e.moveit=Bridge()
    data=dict(op='intent',seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),
        source='keyboard',client_sample_ms=0,intent=intent().model_dump())
    if case=='unprepared':e.session.ready=False
    elif case=='neutral':data['intent']['linear_m_s']=[0]*3
    elif case=='gripper_only':data['intent'].update(linear_m_s=[0]*3,gripper_rate_s=.02)
    elif case=='model_failed':e.moveit.info.update(ready=False,error='worker unavailable')
    elif case=='stop_pending':e.stop_pending=object()
    elif case=='stop_failed':e.stop_failed=True
    elif case=='feedback_stale':d.data['age_ms']=101
    elif case=='feedback_reconnected':d.data['generation']='b'
    elif case=='alarm':d.data['status']['error_status']=1
    elif case=='disabled':d.data['status'].update(enable_status=0,robot_mode=4)
    elif case=='camera_stale':e.camera=lambda:dict(valid=False)
    elif case=='duplicate':data['seq']=e.session.sequence
    elif case=='stale_echo':now[0]+=251_000_000
    elif case=='source_change':data['source']='gamepad'
    elif case=='out_of_range':data['intent']['linear_m_s']=[.1,0,0]
    e.receive(data)
    assert not [c for c in e.moveit.calls if c!='stop']
    assert not d.calls


def test_stop_pending_does_not_restart_candidate_generation_in_tick():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    d.auto_ack=False;send();assert e.stop_pending is not None
    e.moveit.calls.clear()
    send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert not e.moveit.calls and [c[0] for c in d.calls]==['ServoJ','Stop']
    assert e.session.ready


def test_discarded_echo_cannot_replace_candidate_provenance_or_delay_release():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);accepted=e.session.intent_sequence
    e.moveit.row=row(now[0],accepted);step()
    stale=dict(op='intent',seq=e.session.sequence+1,echo_ns=0,source='keyboard',
        client_sample_ms=0,intent=intent(linear_m_s=[-.002,0,0]).model_dump())
    e.moveit.calls.clear();e.receive(stale)
    assert not e.moveit.calls and e.session.ready
    step()
    assert e.moveit.calls[-1][1]==accepted
    assert e.moveit.calls[-1][0].linear_m_s==[.002,0,0]
    stale['seq']+=1;stale['intent']['deadman']=False;e.receive(stale)
    assert d.calls[-1][0]=='Stop' and e.moveit.row is None and e.session.ready


def test_gateway_streams_only_moveit_candidate_not_native_cartesian_approximation():
    e,d,send,step,now=setup();e.moveit=Bridge()
    for n in range(5):
        send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0],e.session.sequence);step()
        d.data['status'].update(robot_mode=7,run_queued_cmd=1)
    assert [c[0] for c in d.calls]==['ServoJ']*5
    np.testing.assert_allclose(d.calls[-1][1][0],[0,0,0,0,0,math.degrees(.0001)])
    assert e.candidate['input_sequence']==e.session.sequence
    send();assert d.calls[-1][0]=='Stop' and e.moveit.row is None and e.session.ready


def test_moveit_failure_cannot_disable_raw_feedback_or_stop_confirmation():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    e.moveit.info=dict(ready=False,error='KDL terminated');step()
    assert d.calls[-1][0]=='Stop' and not e.session.ready
    reads=d.reads;step(10)
    assert d.reads>reads and e.stop_acknowledged


def test_coalesced_solver_rejection_still_stops_before_a_reset_candidate():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    # The producer already reset status to zero. The immutable refusal must
    # still win over an otherwise consumable candidate at the physical writer.
    e.moveit.info.update(status=0,last_rejection_detail=dict(id=20,epoch=1,status=5,reason='official_solver_blocked'))
    e.moveit.row=row(now[0]);step()
    assert [c[0] for c in d.calls]==['ServoJ','Stop']
    assert e.session.ready and e.session.input_paused_ns is not None
    for _ in range(10):
        send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    assert [c[0] for c in d.calls]==['ServoJ','Stop']


def test_tracking_stop_receipt_and_feedback_keep_advancing_after_latched_error():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    d.data['status']['joint_state']['q_target'][4]=-3
    for _ in range(11):
        send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    assert d.calls[-1][0]=='Stop' and not e.session.ready
    assert e.joint_tracking.last_detail['reason']=='joint_tracking_sustained'
    reads=d.reads;step(5)
    assert e.stop_pending is None and e.stop_acknowledged and d.reads>reads


def test_actual_writer_rejects_cross_reset_jump_even_with_successful_solver_status():
    e,d,send,step,now=setup(motion_profile='commissioning');e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    # Both actual and controller target could be reset, so a residual-error
    # check alone is insufficient. Preserve the last actual write as well.
    d.data['status']['joint_state']['q_actual'][4]=7
    d.data['status']['joint_state']['q_target'][4]=7
    e.moveit.row=row(now[0]);e.moveit.row['joints_rad'][4]=math.radians(7)
    step()
    assert len([c for c in d.calls if c[0]=='ServoJ'])==1
    assert d.calls[-1][0]=='Stop' and e.session.ready and e.session.input_paused_ns is not None
    assert e.joint_tracking.last_detail['reason']=='joint_command_discontinuity'


def test_moveit_candidate_starvation_stops_and_preserves_explicit_recovery():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    for _ in range(4):send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert e.session.ready and e.session.input_paused_ns is not None and d.calls[-1][0]=='Stop'
    step(110);send();assert e.session.ready and e.session.input_paused_ns is None


@pytest.mark.parametrize('status',[2,5,6])
def test_upstream_directional_halt_stops_once_without_locking_retreat(status):
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    # Even a still-buffered candidate must not be sent after a halt status.
    e.moveit.info['status']=status;e.moveit.row=row(now[0]);step()
    assert e.session.ready and [c[0] for c in d.calls]==['ServoJ','Stop']
    for _ in range(8):send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert e.session.ready and e.stop_acknowledged
    assert e.moveit.calls.count('stop')==1
    send();step();send(deadman=True,joint_rad_s=[0,0,0,0,0,.01])
    e.moveit.info['status']=0;e.moveit.row=row(now[0],e.session.sequence);step()
    assert e.session.ready and d.calls[-1][0]=='ServoJ'


def test_directional_halt_never_hides_raw_feedback_loss():
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    e.moveit.info['status']=2;step();d.data['connected']=False;step()
    assert not e.session.ready and d.calls[-1][0]=='Stop'


@pytest.mark.parametrize('code,text',[
    ('sweep_travel_limit','目标与反馈间扫掠过大'),
    ('sweep_budget','几何检查超时'),
    ('collision','模型碰撞边界'),
])
def test_geometry_rejection_reports_actual_cause_and_retains_evidence(code,text):
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0]);step()
    check=dict(arm_safe=False,arm_reason=code,arm_travel_m=.041)
    e.moveit.info.update(status=5,gripper_check=check);step()
    assert text in e.reason and e.session.ready and d.calls[-1][0]=='Stop'
    records=[x for x in e.events if x['event']=='model_direction_blocked']
    assert len(records)==1 and records[0]['gripper_check']==check
    step();assert len([x for x in e.events if x['event']=='model_direction_blocked'])==1


def test_model_derivation_uses_only_basis_and_rejects_asymmetric_input():
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('builder',root/'scripts/build_controller_model.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    file=root.parent/'docs/teleop-rebuild/evidence/official-model-audit-v18/summary.json'
    data=json.loads(file.read_text())['samples'];derived=module.derive_basis(data)
    assert derived['flange_axial']==pytest.approx(.0695)
    assert derived['shoulder_height']==pytest.approx(.1652)
    data[-1]['controller_pose_mm_deg']=[999]*6
    assert module.derive_basis(data)==derived
    bad=deepcopy(data);bad[4]['controller_pose_mm_deg'][0]+=.01
    with pytest.raises(ValueError,match='symmetry'):module.derive_basis(bad)
    with pytest.raises(ValueError,match='basis'):module.derive_basis(data[1:])
