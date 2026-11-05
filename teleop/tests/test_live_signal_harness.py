"""Input-driven live acceptance against a substitute only; no sockets/device IO."""
import copy
import json
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from live_signal_acceptance import LiveSignals,axis_cases,final_stop_status,transmitted_gripper_goal
from test_hardware_control import setup

ROOT=Path(__file__).resolve().parents[2]
FIXTURES=json.loads((ROOT/'docs/evidence/teleop_local_kinematics_20261005/production_mapping.json').read_text(encoding='utf-8'))


def test_late_stop_ack_after_owner_exit_is_preserved():
    old=dict(session='owned',stop_requested_ns=100,status_revision_ns=120,stop_acknowledged=False)
    observed=dict(session=None,stop_requested_ns=100,status_revision_ns=200,stop_acknowledged=True)
    assert final_stop_status(old,observed) is observed


@pytest.mark.parametrize('change',[dict(session='another'),dict(stop_requested_ns=101),dict(status_revision_ns=119)])
def test_different_or_older_stop_cannot_confirm_this_request(change):
    old=dict(session='owned',stop_requested_ns=100,status_revision_ns=120,stop_acknowledged=False)
    observed=dict(session=None,stop_requested_ns=100,status_revision_ns=200,stop_acknowledged=True,**{})
    observed.update(change)
    assert final_stop_status(old,observed) is old


def test_selected_axis_suite_declares_exact_partial_coverage():
    cases=axis_cases('frames-fine','tool','fine')
    assert len(cases)==6 and all(f=='tool' and fine for f,fine,_,_ in cases)
    assert len(axis_cases('frames-fine'))==24 and len(axis_cases('axes'))==6
    with pytest.raises(ValueError):axis_cases('zero','tool')
    with pytest.raises(ValueError):axis_cases('axes','tool')


def test_each_independent_axis_is_proven_once_without_claiming_other_axes():
    selected=[]
    for frame in ('base','tool'):
        for fine in ('normal','fine'):
            for mode in ('translation','rotation'):
                for axis in range(3):
                    cases=axis_cases('frames-fine',frame,fine,mode,axis)
                    assert cases==[(frame,fine=='fine',mode,axis)]
                    selected.extend(cases)
    assert set(selected)==set(axis_cases('frames-fine')) and len(selected)==24
    with pytest.raises(ValueError):axis_cases('zero',mode_filter='rotation')
    with pytest.raises(ValueError):axis_cases('axes',axis_filter=3)


class SubstituteSignals(LiveSignals):
    def __init__(self):
        self.fixtures=FIXTURES;self.clock,self.device,self.core=setup('live')
        self.status=self.core.status();self.latest={};self.checks=[];self.seq=0
        self.action_ticks=0;self.candidate_ticks=0
    async def rpc(self,data):
        data=dict(data,session=self.core.owner)
        if data['operation']=='input':
            self.seq+=1;data.update(seq=self.seq,lease=self.core.lease)
        self.status=self.core.command(data,self.device.state(),self.clock());return dict(ok=True,status=self.status)
    async def phase(self,seconds,signal,require_armed=False):
        for _ in range(max(1,round(seconds/.05))):
            await self.rpc(copy.deepcopy(signal));self.clock.step()
            result=self.core.tick(self.device.state(),{'rgb':{'x5_receive_monotonic_ns':self.clock()}})
            self.status=result['status'];self.latest=dict(control=dict(tick=result),sources=dict(robot=self.device.state()))
            self.action_ticks+=int(bool(result['effective_action']));self.candidate_ticks+=int(bool(result['candidate_action']))
            if require_armed and not self.core.armed:raise RuntimeError('Substitute control stopped: '+self.core.reason)


@pytest.mark.parametrize('source,mode,frame,axis,amount',[
    ('keyboard','translation','base',0,.001),('gamepad','rotation','tool',2,np.radians(.25))])
def test_production_mapped_input_round_trips_against_substitute(source,mode,frame,axis,amount):
    import asyncio
    signals=SubstituteSignals()
    async def run():
        await signals.phase(5.5,signals.neutral(source))
        await signals.round_trip(source,mode,frame,axis,amount)
    asyncio.run(run())
    assert len(signals.checks)==1 and signals.checks[0]['passed']
    assert signals.action_ticks>10 and signals.device.stop_count==2
    assert signals.core.armed and signals.status['stopped_feedback']


def test_substitute_ik_fault_never_retries_or_sends_a_target():
    import asyncio
    signals=SubstituteSignals()
    async def run():
        await signals.phase(5.5,signals.neutral('gamepad'));signals.device.ik_error=True
        with pytest.raises(RuntimeError,match='stopped'):
            await signals.round_trip('gamepad','translation','base',0,.001)
    asyncio.run(run())
    assert signals.device.servo_count==0 and not signals.checks and not signals.core.armed
    assert signals.device.stop_count==1


def test_failed_shadow_refused_before_any_output_or_connection(tmp_path):
    shadow=tmp_path/'failed.json';shadow.write_text(json.dumps(dict(passed=False)))
    with pytest.raises(ValueError,match='incomplete/failed'):LiveSignals(FIXTURES,tmp_path/'live',shadow)
    assert not (tmp_path/'live').exists()


@pytest.mark.parametrize('initialized,faulted',[(False,False),(True,True)])
def test_independent_gripper_cycle_rejects_unknown_or_faulted_initialization(initialized,faulted):
    import asyncio
    signals=SubstituteSignals();operations=[]
    signals.status.update(gripper_initialized=initialized,gripper_faulted=faulted)
    async def rpc(data):operations.append(data['operation'])
    async def phase(*args):pass
    signals.rpc=rpc;signals.phase=phase
    with pytest.raises(ValueError,match='completed nonfaulted initialization'):
        asyncio.run(signals.gripper(initialized=True))
    assert operations==['stop'] and signals.device.servo_count==0


def test_gripper_initialization_observes_five_seconds_before_preparation_without_retry():
    import asyncio
    signals=SubstituteSignals();events=[]
    async def rpc(data):
        events.append(data['operation'])
        if data['operation']=='initialize-gripper':
            signals.status.update(commissioning=dict(state='complete'))
    async def phase(seconds,signal,*args):events.append(seconds)
    async def prepare(*args):
        events.append('prepare');raise RuntimeError('End substitute before active inputs')
    signals.rpc=rpc;signals.phase=phase;signals.prepare=prepare
    with pytest.raises(RuntimeError,match='End substitute'):
        asyncio.run(signals.gripper())
    assert events==['stop',.3,'initialize-gripper',.1,5.5,'prepare']
    assert signals.device.servo_count==0


def test_gripper_settling_counts_distinct_replies_and_sends_only_neutral():
    import asyncio
    signals=SubstituteSignals();inputs=[]
    replies=[(1,.18),(2,.15),(2,.15),(3,.15),(4,.15)]
    async def phase(seconds,signal,require_armed=False):
        inputs.append(copy.deepcopy(signal));seq,opening=replies[len(inputs)-1]
        signals.latest=dict(sources=dict(gripper=dict(valid=True,seq=seq,state=0,fault=0,opening=opening)))
    signals.phase=phase
    result=asyncio.run(signals.gripper_settled(signals.neutral(),.15))
    assert len(inputs)==5 and result['feedback']['seq']==4
    assert all(not s['deadman'] and not s['gripper_rate'] and not any(s['twist']) for s in inputs)
    assert signals.device.servo_count==0 and signals.device.stop_count==0


def test_gripper_at_target_state_without_position_arrival_remains_failed():
    import asyncio
    signals=SubstituteSignals();calls=[]
    async def phase(seconds,signal,require_armed=False):
        calls.append(signal)
        signals.latest=dict(sources=dict(gripper=dict(valid=True,seq=len(calls),state=0,fault=0,opening=47/255)))
    signals.phase=phase
    with pytest.raises(ValueError,match='six-second'):
        asyncio.run(signals.gripper_settled(signals.neutral(),.15))
    assert len(calls)==120 and signals.device.servo_count==0


def test_gripper_endpoint_uses_sent_bytes_and_retains_nominal_quantization():
    tick={'last_accepted_action':dict(gripper_acceptance='request_associated_feedback',gripper=41/255,
        requested_gripper=.15,gripper_feedback={'command_position_raw':41})}
    profile=dict(minimum_target_delta_counts=4,gripper_s=.1)
    endpoint=transmitted_gripper_goal(tick,.15,profile)
    assert endpoint['effective_gripper_target']==41/255 and endpoint['nominal_gripper_target']==.15
    tick['last_accepted_action']['gripper']=.15
    with pytest.raises(ValueError,match='transmitted byte'):transmitted_gripper_goal(tick,.15,profile)
    tick['last_accepted_action'].update(gripper=48/255,gripper_feedback={'command_position_raw':48})
    with pytest.raises(ValueError,match='resolution'):transmitted_gripper_goal(tick,.15,profile)


@pytest.mark.parametrize('source',['gamepad','keyboard'])
def test_gripper_endpoint_approach_uses_existing_precision_mapper(source):
    signals=SubstituteSignals()
    for goal,fine in ((.2,False),(.33,True),(.35,True)):
        signals.latest={'control':{'tick':{'candidate_action':{'gripper':goal}}}}
        actual=signals.gripper_signal(source,1,.35)
        assert actual==signals.row(source,'translation','base',3,1,fine)
        assert actual['fine'] is fine and actual['deadman']


def test_archive_start_wait_keeps_only_fresh_neutral_inputs(monkeypatch):
    import asyncio,time
    import live_signal_acceptance as module
    signals=SubstituteSignals();sent=[];requests=[];signals.remote_error=None
    def delayed(path,data):
        requests.append((path,data));time.sleep(.32);return {'episode':'owned'}
    async def send(data):sent.append(copy.deepcopy(data))
    monkeypatch.setattr(module,'request',delayed);signals.send=send
    n=signals.neutral('gamepad')
    result=asyncio.run(signals.start_recording('gamepad',n))
    assert result['episode']=='owned' and len(requests)==1 and len(sent)>=5
    assert all(not s['deadman'] and s['neutral'] and not s['gripper_rate'] and not any(s['twist']) for s in sent)


def test_archive_start_input_failure_retains_late_episode_for_abort(monkeypatch):
    import asyncio,time
    import live_signal_acceptance as module
    signals=SubstituteSignals();sent=[];requests=[];stops=[];signals.remote_error=None
    def delayed(path,data):
        requests.append((path,data));time.sleep(.12);return {'episode':'late-owned'}
    async def send(data):
        sent.append(copy.deepcopy(data));raise RuntimeError('input disconnected')
    async def stop(data):stops.append(data)
    monkeypatch.setattr(module,'request',delayed);signals.send=send;signals.rpc=stop
    with pytest.raises(RuntimeError,match='input disconnected'):
        asyncio.run(signals.start_recording('keyboard',signals.neutral('keyboard')))
    assert len(requests)==1 and stops==[{'operation':'stop'}]
    assert signals.episode=='late-owned' and signals.recording
    assert len(sent)==1 and not sent[0]['deadman'] and sent[0]['neutral']


def test_normal_release_preserves_last_can_goal_only_with_new_matching_hold_feedback():
    previous=dict(gripper_acceptance='request_associated_feedback',gripper=41/255,
        requested_gripper=.15,gripper_feedback={'command_position_raw':41})
    tick={'last_accepted_action':dict(gripper_acceptance='suppressed_due_to_deadman_release',
        requested_gripper=.15,gripper=None)}
    profile=dict(minimum_target_delta_counts=4,gripper_s=.1)
    fresh=dict(valid=True,state=0,fault=0,command_position_raw=41,command_opening=41/255,
        requested_opening=.15,source='can1 request-associated feedback, not cryptographic authentication')
    result=transmitted_gripper_goal(tick,.15,profile,previous_action=previous,holding_feedback=fresh)
    assert result['command_position_raw']==41 and tick['last_accepted_action']['gripper'] is None
    for bad in (dict(fresh,valid=False),dict(fresh,source='cached unknown'),dict(fresh,state=2)):
        with pytest.raises(ValueError,match='matching hold'):
            transmitted_gripper_goal(tick,.15,profile,previous_action=previous,holding_feedback=bad)
    held=dict(fresh,command_position_raw=39,command_opening=39/255)
    assert transmitted_gripper_goal(tick,.15,profile,previous_action=previous,holding_feedback=held)['command_position_raw']==39
    assert tick['last_accepted_action']['gripper'] is None
    tick['last_accepted_action']['gripper_acceptance']='uncertain'
    with pytest.raises(ValueError,match='associated feedback'):
        transmitted_gripper_goal(tick,.15,profile,previous_action=previous,holding_feedback=fresh)


def test_settled_waits_for_feedback_with_only_neutral_inputs():
    import asyncio
    signals=SubstituteSignals();calls=[]
    async def phase(seconds,signal,require_armed=False):
        calls.append(copy.deepcopy(signal))
        signals.latest=dict(sources=dict(robot=signals.device.state()))
        signals.status.update(seq=len(calls),stop_requested_ns=signals.clock()-1,
                              stop_acknowledged=len(calls)>=3,stopped_feedback=len(calls)>=5)
    signals.phase=phase
    result=asyncio.run(signals.settled(signals.neutral()))
    assert len(calls)==5 and result['stopped_feedback']
    assert all(not x['deadman'] and x['neutral'] and not any(x['twist']) and not x['gripper_rate'] for x in calls)
    assert signals.device.servo_count==0 and signals.device.stop_count==0


def test_settled_rejects_stale_success_from_the_previous_stop():
    import asyncio
    signals=SubstituteSignals();calls=[]
    signals.status.update(seq=0,stop_requested_ns=1,stop_acknowledged=True,stopped_feedback=True)
    async def phase(seconds,signal,require_armed=False):
        calls.append(signal);signals.latest=dict(sources=dict(robot=signals.device.state()))
        if len(calls)==4:
            signals.status.update(seq=4,stop_requested_ns=signals.clock()-1)
    signals.phase=phase
    result=asyncio.run(signals.settled(signals.neutral()))
    assert len(calls)==4 and result['stop_requested_ns']>1


def test_settled_never_continues_when_stationary_feedback_is_missing():
    import asyncio
    signals=SubstituteSignals();calls=[]
    async def phase(seconds,signal,require_armed=False):
        calls.append(copy.deepcopy(signal))
        signals.status.update(stop_acknowledged=True,stopped_feedback=False)
    signals.phase=phase
    with pytest.raises(RuntimeError,match='stationary feedback'):
        asyncio.run(signals.settled(signals.neutral()))
    assert len(calls)==60 and all(not x['deadman'] for x in calls)
    assert signals.device.servo_count==0 and signals.device.stop_count==0


@pytest.mark.parametrize('mode,frame,fine',[(m,f,s) for m in ('translation','rotation') for f in ('base','tool') for s in (False,True)])
def test_three_production_mapped_axes_round_trip_without_direct_joint_targets(mode,frame,fine):
    import asyncio,subprocess
    signals=SubstituteSignals()
    generated=subprocess.run(['node',str(ROOT/'teleop/web/scripts/synthetic-inputs.mjs')],capture_output=True,text=True,check=True)
    signals.fixtures=json.loads(generated.stdout)
    async def run():
        await signals.phase(5.5,signals.neutral('keyboard'))
        await signals.multi_round_trip('keyboard',mode,frame,fine)
    asyncio.run(run())
    assert signals.checks[0]['passed'] and signals.checks[0]['multi_axes']==[0,1,2]
    assert signals.device.stop_count==2 and signals.core.armed
