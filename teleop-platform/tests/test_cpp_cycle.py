from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import pytest
from e6_phosphobot.cpp_cycle import CppCycle
from e6_phosphobot.jazzy_cpp_worker import verified_binary
from e6_phosphobot.session_runner import motion_engine_option


def context(now, epoch=1, active=True, generation=1):
    intent=dict(linear_m_s=[0]*3,angular_rad_s=[0]*3,joint_rad_s=[0]*5+[.01],
                frame='base',deadman=True,gripper_rate_s=0)
    actual=dict(connected=True,received_ns=now,generation=generation,
                status=dict(joint_state=dict(q_actual=[0]*6),
                            tool_vector_actual=dict(x=0,y=0,z=400,rx=0,ry=0,rz=0)))
    command=dict(epoch=epoch,sequence=10,received_ns=now,intent=intent) if active else None
    return dict(actual=actual,command=command,epoch=epoch)


def response(cycle, reset=None):
    p=cycle.pending
    previous=p['previous']
    reset=(not p['active'] or not previous or previous['epoch']!=p['epoch']) if reset is None else reset
    seed=list(p['q']) if reset else list(previous['q'])
    q=list(seed)
    if p['active']:q[-1]+=.0001
    return dict(seq=p['id'],epoch=p['epoch'],active=p['active'],reset=reset,status=0,
                compute_us=20,callback_us=50,seed_positions=seed,
                seed_velocities=[0]*6 if reset else list(previous['v']),
                joints_rad=q,velocities_rad_s=[0]*5+[.002 if p['active'] else 0],
                actual_fk=dict(position_m=[0,0,.4],quaternion_xyzw=[0,0,0,1]),
                target_fk=dict(position_m=[0,0,.4],quaternion_xyzw=[0,0,0,1]))


def initialized():
    c=CppCycle();now=1_000_000_000
    c.update(context(now,active=False));assert c.request(now)
    assert c.complete(response(c),now+1_000_000) is None
    assert c.info['ready'] and not c.reset_required
    return c,now+2_000_000


def pending():
    c,now=initialized();c.update(context(now));assert c.request(now)
    assert c.pending['active']
    return c,now


def test_initial_state_is_observed_before_any_active_solver_step():
    c=CppCycle();now=1_000_000_000;c.update(context(now))
    request=c.request(now).split()
    assert request[2]=='0'
    assert c.complete(response(c),now+1) is None and c.info['ready']
    assert c.request(now+2).split()[2]=='1'


@pytest.mark.parametrize('startup_ms',[77.970166,352.122323,352.250575])
@pytest.mark.parametrize('feedback_arrives',[False,True])
def test_slow_bootstrap_cannot_turn_a_buffered_input_into_a_motion_target(startup_ms,feedback_arrives):
    c=CppCycle();now=1_000_000_000
    # Even if input arrives during startup, the first query is observation.
    ctx=context(now);c.update(ctx)
    assert c.request(now).split()[2]=='0'
    reply=response(c);reply['callback_us']=startup_ms*1000
    finish=now+int(startup_ms*1_000_000)
    if feedback_arrives:
        ctx=deepcopy(ctx);ctx['actual']['received_ns']=finish
        c.update(ctx)
    assert c.complete(reply,finish) is None
    assert c.snapshot(finish)['candidate'] is None and c.latest is None
    if not feedback_arrives and startup_ms>=100:
        assert not c.snapshot(finish)['info']['ready']
    if startup_ms>=250:
        assert not c.active(finish)
    # A fresh neutral observation and a new input start from actual feedback,
    # with no old target as smoother parent.
    fresh=finish+50_000_000
    c.update(context(fresh,epoch=2,active=False))
    assert c.request(fresh).split()[2]=='0'
    assert c.complete(response(c),fresh+1_000_000) is None
    c.update(context(fresh+2_000_000,epoch=3))
    assert c.request(fresh+2_000_000).split()[2]=='1'
    target=c.complete(response(c),fresh+3_000_000)
    assert target['cpp']['reset'] and target['cpp']['parent_id'] is None
    assert target['pre_action_received_ns']==fresh+2_000_000


def test_exact_source_parent_state_deadline_and_twenty_hz_are_retained():
    c,now=pending();reply=response(c);row=c.complete(reply,now+1_000_000)
    assert row['source_input']['sequence']==10 and row['pre_action_received_ns']==now
    assert row['cpp']['parent_id'] is None and row['cpp']['reset']
    assert row['valid_until_ns']==now+100_000_000
    assert c.request(now+49_999_999) is None
    c.update(context(now+50_000_000));assert c.request(now+50_000_000)
    next_row=c.complete(response(c),now+51_000_000)
    assert next_row['cpp']['parent_id']==row['id']
    assert next_row['cpp']['seed_positions']==row['joints_rad']
    assert next_row['input_association']=='official_cpp_step_with_smoother_seed'
    assert next_row['trajectory_sources']==[]  # Do not invent a ROS window.


@pytest.mark.parametrize('case',['released','new_epoch','input_expired','feedback_expired',
                                  'generation','late','deadman','neutral'])
def test_inflight_result_is_discarded_when_original_conditions_no_longer_hold(case):
    c,now=pending();reply=response(c);ctx=deepcopy(c.context);finish=now+1_000_000
    if case=='released':ctx['command']=None
    elif case=='new_epoch':ctx=context(now,epoch=2)
    elif case=='input_expired':ctx['command']['received_ns']=now-250_000_000
    elif case=='feedback_expired':ctx['actual']['received_ns']=now-100_000_000
    elif case=='generation':ctx['actual']['generation']=2
    elif case=='late':finish=now+50_000_000
    elif case=='deadman':ctx['command']['intent']['deadman']=False
    elif case=='neutral':ctx['command']['intent']['joint_rad_s']=[0]*6
    c.update(ctx)
    assert c.complete(reply,finish) is None
    assert c.latest is None and c.reset_required
    assert c.snapshot(finish)['candidate'] is None


def test_rejected_late_state_requires_neutral_reset_before_fresh_command():
    c,now=pending();assert c.complete(response(c),now+50_000_000) is None
    c.update(context(now+60_000_000));assert c.request(now+60_000_000).split()[2]=='0'
    assert c.complete(response(c),now+61_000_000) is None
    c.update(context(now+120_000_000,epoch=2,active=False))
    assert c.request(now+120_000_000).split()[2]=='0'
    assert c.complete(response(c),now+121_000_000) is None
    c.update(context(now+122_000_000,epoch=3))
    assert c.request(now+122_000_000).split()[2]=='1'
    assert c.complete(response(c),now+123_000_000)['cpp']['reset']


def test_collision_rejection_stays_visible_during_unchanged_held_input():
    c,now=pending();r=response(c);r['status']=5
    assert c.complete(r,now+1_000_000) is None
    for tick in range(1,21):
        t=now+tick*50_000_000;c.update(context(t))
        assert c.request(t).split()[2]=='0'
        assert c.complete(response(c),t+1_000_000) is None
        assert c.snapshot(t)['candidate'] is None and c.info['status']==5
    assert c.info['last_rejection_detail']['id']==1 or c.info['last_rejection_detail']['id']==2


@pytest.mark.parametrize('status',[-1,2,5,6])
def test_official_boundaries_never_produce_an_executable_zero_target(status):
    c,now=pending();r=response(c);r['status']=status
    # Retreat diagnostics are audit data, never permission to override a stop.
    r['collision_retreat']=dict(boosted=True,verified=True,selected_scale=1.)
    assert c.complete(r,now+1) is None
    assert c.info['status']==status and c.reset_required


def test_retreat_trace_is_preserved_without_extending_the_command_deadline():
    c,now=pending();r=response(c)
    r['collision_retreat']=dict(boosted=True,verified=True,baseline_scale=.03,selected_scale=1.,reason='retreat')
    row=c.complete(r,now+1)
    assert row['cpp']['collision_retreat']==r['collision_retreat']
    assert row['valid_until_ns']==now+100_000_000
    r['collision_retreat']['verified']=False
    assert row['cpp']['collision_retreat']['verified'] is True
    c,now=pending();r=response(c)
    r['collision_retreat']=dict(boosted=True,verified=True,selected_scale=1.)
    assert c.complete(r,now+50_000_000) is None
    assert c.info['last_rejection']=='solver_deadline'


@pytest.mark.parametrize('elapsed_us',[42_000,42_296,49_999,float('nan'),float('inf'),-1])
def test_completed_retreat_proof_does_not_override_callback_cutoff(elapsed_us):
    c,now=pending();r=response(c);r['callback_us']=elapsed_us
    r['collision_retreat']=dict(boosted=True,verified=True,selected_scale=1.)
    assert c.complete(r,now+49_000_000) is None
    assert c.info['last_rejection']=='solver_callback_deadline' and c.reset_required


@pytest.mark.parametrize('case',['sequence','nan','bad_quaternion','bad_status','parent','reset_source'])
def test_malformed_or_unattributed_results_are_rejected(case):
    c,now=pending();r=response(c)
    if case=='sequence':r['seq']+=1
    elif case=='nan':r['joints_rad'][0]=math.nan
    elif case=='bad_quaternion':r['target_fk']['quaternion_xyzw']=[0,0,0,0]
    elif case=='bad_status':r['status']=True
    elif case=='parent':r['reset']=False
    elif case=='reset_source':r['seed_positions'][0]=.1
    with pytest.raises(ValueError):c.complete(r,now+1)


def test_model_mismatch_and_stale_feedback_do_not_kill_raw_reader():
    c,now=pending();r=response(c);r['actual_fk']['position_m'][0]=.01
    assert c.complete(r,now+1) is None and not c.info['ready']
    c.update(context(now+100_000_000));assert c.request(now+100_000_000)
    c.complete(response(c),now+100_000_001)
    assert c.info['ready']
    assert c.request(now+201_000_000) is None
    assert not c.snapshot(now+201_000_000)['info']['ready']
    # Device reader/Stop are never dependencies of this object.
    assert not hasattr(c,'reader') and not hasattr(c,'dashboard')


def test_binary_checksum_and_bundle_boundary_are_enforced(tmp_path):
    binary=tmp_path/'adapter';binary.write_bytes(b'test executable fixture')
    manifest=dict(binary='adapter',sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    path=tmp_path/'cpp-adapter.json';path.write_text(json.dumps(manifest))
    assert verified_binary(tmp_path)[0]==binary
    binary.write_bytes(b'changed')
    with pytest.raises(ValueError,match='checksum'):verified_binary(tmp_path)
    manifest['binary']='../outside';path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='outside'):verified_binary(tmp_path)


def test_launcher_api_is_explicit_and_default_route_is_unchanged():
    base=dict(rootfs='/rootfs',motion_engine='moveit_servo')
    assert motion_engine_option(base)==['--moveit-bundle',str(Path('/rootfs')/'qualification')]
    assert motion_engine_option(dict(base,moveit_api='cpp'))[-2:]==['--moveit-api','cpp']
    for config in (dict(base,moveit_api='invalid'),dict(base,motion_engine='native_servop',moveit_api='cpp')):
        with pytest.raises(ValueError):motion_engine_option(config)


def test_async_twenty_hz_consumption_uses_original_source_deadlines_not_a_second_lease():
    from e6_phosphobot.moveit_bridge import MoveItBridge
    from e6_phosphobot.intent import Intent
    c,now=pending();row=c.complete(response(c),now+5_000_000)
    clock=[now+55_000_000]
    bridge=MoveItBridge('.',None,clock=lambda:clock[0])
    bridge.submit(Intent(**c.context['command']['intent']),10)
    assert bridge.epoch==row['epoch']
    bridge.latest=row
    assert bridge.take()==row  # Next C++ result may arrive a few ms later.
    bridge.latest=row
    assert bridge.take() is None  # Never duplicate/replay to fill a gap.
    row=dict(row,id=row['id']+1)
    clock[0]=now+100_000_001;bridge.latest=row
    assert bridge.take() is None  # Original feedback is now too old.


@pytest.mark.parametrize('source,age,expected', [('actual',40_000_000,60_000_000),('command',245_000_000,5_000_000)])
def test_original_source_age_is_never_refreshed_at_computation(source,age,expected):
    c,now=initialized();ctx=context(now)
    ctx[source]['received_ns']-=age;c.update(ctx);assert c.request(now)
    row=c.complete(response(c),now+1)
    assert row['valid_until_ns']==now+expected


def test_cpp_runtime_manifest_is_required_and_added_only_for_that_api():
    from e6_phosphobot.session_runner import runtime_artifacts
    original={'qualification/model':'abc'}
    assert runtime_artifacts(dict(runtime_manifest=original))==original
    with pytest.raises(ValueError):runtime_artifacts(dict(moveit_api='cpp'))
    extra={'qualification/cpp-adapter.json':'def','qualification/cpp/servo_cpp_adapter':'ghi'}
    assert runtime_artifacts(dict(moveit_api='cpp',runtime_manifest=original,cpp_runtime_manifest=extra))==dict(original,**extra)
    assert original=={'qualification/model':'abc'}


@pytest.mark.parametrize('case',['valid','deadline','actual','time','reset','association'])
def test_independent_record_audit_rejects_corrupt_cpp_provenance(case):
    import importlib.util
    path=Path(__file__).parents[1]/'scripts/audit_moveit_page.py'
    spec=importlib.util.spec_from_file_location('cpp_record_audit',path)
    audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    c,now=pending();row=c.complete(response(c),now+1)
    raw={now:([0.]*6,[0,0,400,0,0,0])}
    if case=='deadline':row['valid_until_ns']+=1
    elif case=='actual':row['cpp']['actual_q_rad'][0]=.01
    elif case=='time':row['computed_ns']=now+50_000_000
    elif case=='reset':row['cpp']['seed_velocities'][0]=.1
    elif case=='association':row['input_association']='fake'
    errors=audit.cpp_errors(row,raw,{row['id']:row})
    assert bool(errors)==(case!='valid')
