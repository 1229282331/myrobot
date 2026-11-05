"""CAN parameters, mapped rates and operator startup stay bound together."""
import pytest
from e6_teleop.gripper_profile import gripper_profile
from e6_teleop.tunnel import Tunnel
from e6_observer.gripper_profile import gripper_profile as gateway_profile
from e6_observer.control_guardian import GripperLink


@pytest.mark.parametrize('name',['plan32','empty64-medium'])
def test_same_settings_on_pc_and_gateway(name):
    assert gripper_profile(name)==gateway_profile(name)


def test_initial_arm_caps_and_medium_gripper_are_independent():
    tunnel=Tunnel('127.0.0.1',control_stage='shadow',gripper_command_profile='empty64-medium')
    assert tunnel.tested_limits()=={'gripper_s':.1}
    assert Tunnel('127.0.0.1',control_stage='shadow').tested_limits()=={}


def test_operator_cannot_change_accepted_profile_or_rate():
    with pytest.raises(ValueError,match='differs'):
        Tunnel('127.0.0.1',control_stage='live',control_role='operator',acceptance_profile={'gripper_command_profile':'plan32'},gripper_command_profile='empty64-medium')
    t=Tunnel('127.0.0.1',control_stage='live',control_role='operator',acceptance_profile={'media_hz':10,'limits':{'gripper_s':.1}})
    with pytest.raises(ValueError,match='exceeds'):t.tested_limits()


def test_observation_cannot_select_command_profile():
    with pytest.raises(ValueError,match='requires hardware-control'):
        Tunnel('127.0.0.1',gripper_command_profile='empty64-medium')


@pytest.mark.parametrize('name,force,velocity',[('plan32',32,10),('empty64-medium',64,25)])
def test_exchange_uses_fixed_parameters_and_keeps_request_feedback(name,force,velocity):
    # Exercise the production exchange without USB/CAN setup or IO.
    class Can:
        @staticmethod
        def encode_command(target,parameters,bounds):
            assert target==35 and bounds==(15,60)
            assert parameters==dict(force=force,velocity=velocity,acceleration=25,deceleration=25)
            return bytes((0,89,velocity,force,25,25,0,0))
    class Channel:
        def exchange(self,command):
            assert command==bytes((0,89,velocity,force,25,25,0,0))
            link.report['events'].append(dict(kind='tx',send_confirmed=True))
            return dict(position_raw=89,host_perf_counter_ns=123,state=0,fault=0)
    link=GripperLink.__new__(GripperLink);link.can=Can;link.channel=Channel()
    settings=gripper_profile(name)
    link.command_parameters={k:settings[k] for k in ('velocity','force','acceleration','deceleration')}
    link.report={'events':[]};link.request_count=0;link.tx_count=0
    feedback=link.exchange(.35)
    assert link.tx_count==1 and feedback['seq']==1
    assert feedback['opening']==89/255 and feedback['x5_receive_monotonic_ns']==123
    assert feedback['command_position_raw']==89 and feedback['command_opening']==89/255


def test_small_absolute_updates_accumulate_against_last_sent_target():
    from e6_observer.gripper_profile import select_command
    from omnipicker.tools.verify_can_control import encode_command
    settings=gripper_profile('empty64-medium')
    motor={k:settings[k] for k in ('velocity','force','acceleration','deceleration')}
    previous=encode_command(35,motor)
    sent=[]
    for count in range(88,37,-1):
        candidate=bytes((0,count,25,64,25,25,0,0))
        selected=select_command(candidate,previous,4)
        sent.append(selected[1]);previous=selected
    assert sent[:4]==[89,89,89,85]
    assert set(abs(b-a) for a,b in zip(sent,sent[1:]))<={0,4}
    # Releasing keeps the transmitted target; no queued small remainder.
    for _ in range(20):assert select_command(candidate,previous,4)==previous
    assert abs(previous[1]-38)<4


def test_failed_can_feedback_does_not_commit_a_new_sent_target():
    from omnipicker.tools import verify_can_control as can
    link=GripperLink.__new__(GripperLink);link.can=can
    link.command_parameters=dict(velocity=25,force=64,acceleration=25,deceleration=25)
    link.last_command=can.encode_command(35,link.command_parameters)
    link.minimum_target_delta_counts=4;link.report={'events':[]};link.request_count=0;link.tx_count=0
    previous=link.last_command
    class Channel:
        def exchange(self,command):raise TimeoutError('Uncertain CAN reply')
    link.channel=Channel()
    with pytest.raises(TimeoutError):link.exchange(.15)
    assert link.last_command==previous


def test_effective_gripper_label_is_transmitted_byte_not_requested_increment():
    from test_hardware_control import setup,arm,send,tick
    c,d,g=setup('live');arm(g,d)
    d.grip=lambda target:dict(command_opening=51/255,position_raw=51)
    send(g,d,deadman=True,neutral=False,gripper_rate=.01)
    result=tick(c,d,g)
    assert result['effective_action']['gripper']==51/255
    assert result['candidate_action']['gripper']>51/255
