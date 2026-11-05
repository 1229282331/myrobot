"""Reboot/default payload must never silently become the operating payload."""
import json
from pathlib import Path

import numpy as np
import pytest
from dobot_sdk.api.robot_control import RobotControl
from dobot_sdk.protocol.data_types import MyType
from e6_phosphobot.dashboard import SubmissionPort
from e6_phosphobot.dobot_transport import FeedbackFrame
from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.payload import Payload, FEEDBACK_FIELDS
from e6_phosphobot.session_runner import payload_option
from test_dashboard import setup as socket_setup
from test_enable import disabled
from test_gateway import setup
from test_sdk_upstream import packet


def test_explicit_enable_uses_official_builder_and_full_reply_budget():
    connection, peer = socket_setup(reply_timeout=.001)
    try:
        p = Payload().enable(RobotControl(SubmissionPort(connection)))
        command = 'EnableRobot(0.750000,0.000000,-14.000000,46.000000,0)'
        assert peer.recv(1024) == (command+'\n').encode()
        assert connection.timeout_for(command) == 10.
        p.event['sent_ns'] -= 1_000_000_000
        assert not connection.poll(p)
        peer.sendall(('0,{},'+command+';').encode())
        assert p.done.wait(1) and p.event['accepted']
    finally:
        connection.disconnect(); peer.close()


@pytest.mark.parametrize('value', [
    {}, {'mass_kg': .553}, {'mass_kg': True, 'center_mm': [0,0,0]},
    {'mass_kg': 0., 'center_mm': [0,0,0]}, {'mass_kg': .8, 'center_mm': [0,0,0]},
    {'mass_kg': float('nan'), 'center_mm': [0,0,0]},
    {'mass_kg': .553, 'center_mm': [0,0]},
    {'mass_kg': .553, 'center_mm': [0,float('inf'),0]},
    {'mass_kg': .553, 'center_mm': [0,False,0]},
])
def test_bad_configuration_rejected_before_enable(value):
    with pytest.raises(ValueError):Payload.from_config(value)


def test_configuration_reaches_remote_gateway_and_readiness_check():
    from e6_phosphobot.launcher import gateway_ready
    cfg = json.loads((Path(__file__).parents[1]/'config/workstation.json').read_text())
    expected = Payload().description()
    assert cfg['payload'] == expected
    option = payload_option(cfg)
    assert option[0] == '--payload' and json.loads(option[1]) == expected
    config = {'payload': expected, 'motion_engine': 'native_servop'}
    state = dict(stage='shadow', feedback={'connected': True, 'age_ms': 1}, camera={'valid': True},
                 capabilities=dict(motion_engine='native_servop', servo_gain=500, servo_aheadtime=50,
                                   servo_duration_s=.1, payload=expected))
    assert gateway_ready(state, 'shadow', servo_config=config)
    state['capabilities']['payload'] = dict(mass_kg=.75, center_mm=[0,0,0])
    assert not gateway_ready(state, 'shadow', servo_config=config)


def test_compact_feedback_preserves_actual_sdk_payload_fields():
    wire = np.frombuffer(packet(), dtype=MyType).copy()
    for key, val in zip(('Load','CenterX','CenterY','CenterZ'),(.750,0,-14,46)):
        wire[key] = val
    r = FeedbackReader('no-device')
    r._publish(FeedbackFrame(wire.tobytes(), b'', 1))
    compact, full = r.snapshot()['status'], r.snapshot(full=True)['status']
    assert all(compact[k] == full[k] for k in FEEDBACK_FIELDS)
    assert Payload().mismatch(compact) is None


def test_enable_ack_is_not_payload_confirmation_and_requires_new_stable_window():
    e,d,send,step,now = disabled()
    d.data['status'].update(load=.75, load_center_x=0, load_center_y=0, load_center_z=0)
    assert e.state()['arm']['can_enable']
    send('arm_enable'); step()
    assert d.calls[0][1:3] == ((.750,), dict(centerX=0., centerY=-14., centerZ=46.))
    d.data['status'].update(enable_status=1, robot_mode=5)
    step(110)
    assert e.enable_phase == 'observing' and not e.session.ready
    send(); send('prepare'); assert not e.session.ready
    d.data['status'].update(load=.750, load_center_y=-14., load_center_z=46.)
    step(95); assert e.enable_phase == 'observing'
    step(10); assert e.enable_phase == 'complete' and not e.session.ready
    send(); send('prepare'); assert e.session.ready
    assert [x[0] for x in d.calls] == ['EnableRobot']


def test_wrong_payload_after_enable_times_out_without_retry_or_motion():
    e,d,send,step,now = disabled()
    send('arm_enable'); step()
    d.data['status'].update(enable_status=1, robot_mode=5, load=.553)
    step(405)
    assert e.enable_phase == 'failed' and '负载' in e.enable_reason
    send(); send('prepare'); send('arm_enable')
    assert not e.session.ready and len(d.calls) == 1


@pytest.mark.parametrize('invalid',[.553, None, float('nan')])
def test_wrong_or_unknown_payload_blocks_prepare_and_stops_active_control(invalid):
    e,d,send,step,now = setup()
    send(deadman=True,linear_m_s=[.002,0,0]); step()
    assert d.calls[-1][0] == 'ServoP'
    d.data['status']['load'] = invalid
    step()
    assert d.calls[-1][0] == 'Stop' and not e.session.ready
    send(); send('prepare'); assert not e.session.ready
    d.data['status']['load'] = .750
    step(8); assert not e.session.ready
    assert not any(x[0] in ('EnableRobot','SetPayload') for x in d.calls)


def test_shadow_can_observe_old_payload_without_any_device_commands():
    e,d,send,step,now = setup('shadow')
    d.data['status']['load'] = .553
    send(deadman=True,linear_m_s=[.002,0,0]); step()
    assert e.candidate is not None and not d.calls
    assert e.state()['arm']['payload_matches'] is False


def test_stale_payload_display_is_unknown():
    e,d,send,step,now = setup()
    d.data['age_ms'] = 101
    step()
    assert e.state()['arm']['payload_matches'] is None
