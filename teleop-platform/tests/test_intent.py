from e6_phosphobot.intent import IntentSession
import pytest


def message(session, seq, **kwargs):
    return dict(op='intent', seq=seq,echo_ns=session.clock_tick(),client_sample_ms=seq*50,
                source='keyboard',intent=dict(linear_m_s=[0,0,0],angular_rad_s=[0,0,0],
                joint_rad_s=[0]*6,gripper_rate_s=0,deadman=False,frame='base')) | kwargs


def test_one_hundred_releases_keep_session_ready_and_never_replay():
    s=IntentSession()
    s.receive(message(s,1))
    s.receive(message(s,2,op='prepare',intent=None))
    for i in range(100):
        msg=message(s,3+i*2)
        msg['intent'].update(deadman=True,linear_m_s=[.002,0,0])
        assert s.receive(msg) and s.active().linear_m_s[0]==.002
        assert s.receive(message(s,4+i*2))
        assert s.ready and s.active() is None and s.stop_requested
        s.stop_requested=False


def test_expiry_stops_but_requires_fresh_post_pause_neutral_not_prepare():
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    s.receive(active);now[0]+=251_000_000
    assert s.active() is None and s.ready and s.stop_requested
    assert s.reason=='input_stale' and s.input_paused_ns==now[0]
    now[0]+=1_000_000
    held=message(s,4);held['intent'].update(deadman=True,linear_m_s=[-.002,0,0])
    assert not s.receive(held) and s.active() is None
    assert not s.receive(message(s,5,op='prepare',intent=None))
    assert s.receive(message(s,6)) and s.active() is None and s.reason is None
    held=message(s,7);held['intent'].update(deadman=True,linear_m_s=[-.002,0,0])
    assert s.receive(held) and s.active().linear_m_s==[-.002,0,0]


@pytest.mark.parametrize('change',[
    dict(linear_m_s=[-.002,0,0]),
    dict(linear_m_s=[0,.002,0]),
    dict(linear_m_s=[0,0,0],angular_rad_s=[0,.01,0]),
    dict(linear_m_s=[0,0,0],joint_rad_s=[0,0,0,0,.01,0]),
    dict(linear_m_s=[0,0,0],gripper_rate_s=.02),
    dict(frame='tool'),
    dict(orientation_assist=True,assist_angular_rad_s=.01),
    dict(linear_m_s=[0,0,0],deadman=False),
])
def test_direction_pause_requires_fresh_echo_but_accepts_new_direction(change):
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    s.receive(active);old_echo=s.clock_tick();now[0]+=1
    s.pause_input('solver_direction_blocked',allow_direction_change=True)
    now[0]+=1
    # Even a valid clock-bound sample cannot substitute for seeing the pause.
    s.input_age_bounds.bound=lambda *_:dict(upper_ms=1)
    changed=message(s,4,echo_ns=old_echo)
    changed['intent'].update(active['intent']);changed['intent'].update(change)
    assert not s.receive(changed) and s.input_paused_ns is not None
    changed.update(seq=5,echo_ns=s.clock_tick())
    assert s.receive(changed) and s.ready and s.input_paused_ns is None
    assert s.paused_direction is None and s.reason is None


def test_stricter_pause_cannot_be_downgraded_to_directional_recovery():
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    s.receive(active)
    s.pause_input('solver_direction_blocked',allow_direction_change=True)
    s.pause_input('input_stale')
    s.pause_input('solver_direction_blocked',allow_direction_change=True)
    now[0]+=1
    reverse=message(s,4);reverse['intent'].update(deadman=True,linear_m_s=[-.002,0,0])
    assert not s.receive(reverse) and s.paused_direction is None
    assert s.receive(message(s,5)) and s.ready


def test_late_fresh_held_input_cannot_skip_expiry_between_control_ticks():
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0]);s.receive(active)
    now[0]+=300_000_000
    active=message(s,4);active['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    assert not s.receive(active) and s.active() is None and s.stop_requested
    assert s.ready and s.input_paused_ns==now[0]


def test_stale_or_pre_pause_neutral_cannot_clear_input_pause_and_source_loss_disarms():
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0]);s.receive(active)
    now[0]+=240_000_000;old_tick=s.clock_tick();now[0]+=20_000_000;s.active()
    now[0]+=1_000_000
    assert not s.receive(message(s,4,echo_ns=0))
    assert not s.receive(message(s,5,echo_ns=old_tick))
    assert s.input_paused_ns is not None and s.active() is None
    assert not s.receive(message(s,6,source='gamepad'))
    assert not s.ready and s.reason=='input_source_changed'


def test_released_idle_hiccup_never_replays_but_does_not_lock_session():
    now=[1_000_000_000]
    s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    now[0]+=400_000_000
    assert s.active() is None and s.ready
    stale=message(s,3,echo_ns=1_000_000_000)
    stale['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    assert not s.receive(stale) and s.active() is None
    fresh=message(s,4);fresh['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    assert s.receive(fresh) and s.active() is not None


def test_duplicate_does_not_refresh_input_age_or_reenable():
    now=[1_000_000_000]
    s=IntentSession(clock=lambda:now[0])
    msg=message(s,1)
    s.receive(msg)
    assert not s.receive(msg)
    assert s.received_ns==1_000_000_000 and s.dropped==1
    s.receive(message(s,2,op='stop',intent=None,echo_ns=0))
    assert not s.ready and s.stop_requested


def test_forged_new_timestamp_does_not_renew_server_freshness():
    s=IntentSession(clock=lambda:1_000_000_000)
    assert not s.receive(message(s,1,echo_ns=999_000_000))
    assert s.intent is None


def test_one_stale_echo_keeps_fresh_intent_without_renewing_or_relabelling_it():
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    # v58: the previous input was accepted 3.857 ms before this packet arrived;
    # this packet's clock token aged from 228.203 to 253.441 ms during handling.
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    assert s.receive(active)
    accepted_at=s.received_ns
    now[0]+=29_095_000
    stale=message(s,4,echo_ns=now[0]-253_441_000)
    stale['intent'].update(deadman=True,linear_m_s=[-.002,0,0])
    assert not s.receive(stale)
    assert s.ready and not s.stop_requested and s.active().linear_m_s==[.002,0,0]
    assert s.sequence==4 and s.intent_sequence==3 and s.received_ns==accepted_at
    # A recovered fresh packet is used normally, without service restart.
    now[0]+=20_000_000
    fresh=message(s,5);fresh['intent'].update(deadman=True,linear_m_s=[-.002,0,0])
    assert s.receive(fresh) and s.intent_sequence==5
    assert s.active().linear_m_s==[-.002,0,0]


def test_stream_of_stale_echoes_cannot_keep_motion_alive():
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    s.receive(active);accepted_at=s.received_ns
    for n in range(1,7):
        now[0]=accepted_at+n*50_000_000
        stale=message(s,3+n,echo_ns=0);stale['intent'].update(deadman=True,linear_m_s=[.002,0,0])
        assert not s.receive(stale) and s.received_ns==accepted_at
    assert s.active() is None and s.ready and s.stop_requested
    now[0]+=1_000_000
    assert s.receive(message(s,10)) and s.ready and s.active() is None


def test_stale_release_stops_immediately_and_does_not_renew_the_lease():
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    active=message(s,3);active['intent'].update(deadman=True,linear_m_s=[.002,0,0])
    s.receive(active);accepted_at=s.received_ns;now[0]+=10_000_000
    assert not s.receive(message(s,4,echo_ns=0))
    assert s.stop_requested and s.active() is None and s.intent_sequence is None
    assert s.ready and s.received_ns==accepted_at


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-.004,.004])
def test_invalid_or_over_capability_is_not_a_target(value):
    s=IntentSession()
    m=message(s,1)
    m['intent']['linear_m_s'][0]=value
    try:
        s.receive(m)
    except ValueError:
        pass
    assert s.active() is None and not s.ready
