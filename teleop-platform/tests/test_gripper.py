import pytest
from e6_phosphobot.gripper import Gripper,opening_raw

class Device:
    def __init__(self):self.targets=[];self.error=None
    def exchange(self,target):
        self.targets.append(target)
        if self.error:raise self.error
        raw=round(target*255)
        return dict(position_percent=raw*100/255,position_raw=raw,state=0,host_perf_counter_ns=1)
    def close(self):pass

def ready():
    d=Device();g=Gripper(device_factory=lambda _:d,clock=lambda:1)
    g.initialize()
    for _ in range(3):g.cycle()
    assert g.snapshot()['valid']
    return g,d

def test_import_and_idle_do_not_open_can_and_initial_position_is_unknown():
    calls=[];g=Gripper(device_factory=lambda _:calls.append(1));g.cycle()
    assert calls==[] and g.snapshot()['actual'] is None

def test_explicit_initialization_and_release_hold_last_target():
    g,d=ready();assert d.targets==[.35]*3
    g.set_target(.30);g.cycle();g.cycle()
    assert d.targets[-2:]==[.30,.30]
    assert g.snapshot()['actual']==pytest.approx(round(.30*255)/255)

def test_fault_stops_can_without_retries_and_keeps_history_unknown():
    g,d=ready();d.error=RuntimeError('stalled')
    g.cycle();count=len(d.targets);g.cycle()
    assert len(d.targets)==count and g.snapshot()['actual'] is None
    assert g.snapshot()['last_observed'] is not None
    with pytest.raises(RuntimeError):g.initialize()
    assert not g.set_target(.25)

def test_expired_gripper_feedback_never_displayed_as_live():
    g,_=ready();g.clock=lambda:201_000_001
    assert not g.snapshot()['valid'] and g.snapshot()['actual'] is None
    assert g.snapshot()['arrived'] is None and g.snapshot()['position_status']=='unknown'


def test_normal_device_state_with_wrong_position_is_not_arrived_or_a_transport_fault():
    g,d=ready();assert g.snapshot()['arrived']
    g.set_target(70/255)
    assert g.snapshot()['position_status']=='pending'
    d.exchange=lambda _:dict(position_percent=74*100/255,position_raw=74,state=0,host_perf_counter_ns=1)
    for _ in range(10):g.cycle()
    s=g.snapshot()
    assert s['phase']=='ready' and s['valid'] and not s['arrived']
    assert s['position_error_counts']==4 and s['position_status']=='not_arrived'
    assert s['target_raw']==s['sent_raw']==70 and s['effective_target']==70/255


def test_late_response_cannot_confirm_a_newer_gripper_target():
    g,d=ready();original=d.exchange
    def exchange(target):
        g.set_target(.30)
        return original(target)
    d.exchange=exchange;g.cycle()
    assert g.snapshot()['position_status']=='pending' and not g.snapshot()['arrived']
    d.exchange=original
    for _ in range(2):g.cycle()
    assert not g.snapshot()['arrived']
    g.cycle();assert g.snapshot()['arrived']


def test_small_inputs_accumulate_without_sending_until_four_counts_then_release_discards_residual():
    g,d=ready()
    for raw in (88,87,86):
        g.set_target(raw/255);g.cycle()
        assert round(d.targets[-1]*255)==89
        assert g.snapshot()['position_status']=='accumulating'
    g.set_target(85/255);g.cycle()
    assert round(d.targets[-1]*255)==85
    g.set_target(83/255);g.hold()
    for _ in range(4):g.cycle()
    assert all(round(t*255)==85 for t in d.targets[-4:])
    assert g.snapshot()['target']==85/255
    # Reverse from the last sent target; no old closing residual survives.
    g.set_target(87/255);g.cycle();assert round(d.targets[-1]*255)==85
    g.set_target(89/255);g.cycle();assert round(d.targets[-1]*255)==89


def test_release_discards_a_complete_unsent_batch_but_preserves_inflight_command():
    g,d=ready();g.set_target(80/255);g.hold();g.cycle()
    assert round(d.targets[-1]*255)==89
    original=d.exchange
    def release_during_exchange(target):
        g.set_target(75/255);g.hold()
        return original(target)
    d.exchange=release_during_exchange;g.set_target(80/255);g.cycle()
    assert g.snapshot()['target']==80/255
    d.exchange=original;g.cycle();assert round(d.targets[-1]*255)==80


@pytest.mark.parametrize('rate',[.025,.05,.1])
def test_continued_input_reaches_both_endpoints_without_sub_four_count_commands(rate):
    g,d=ready()
    from e6_phosphobot.gripper import MINIMUM_OPENING
    for endpoint,direction in ((MINIMUM_OPENING,-1),(1.,1),(MINIMUM_OPENING,-1),(1.,1)):
        previous=round(g.snapshot()['last_sent']*255)
        for _ in range(1000):
            target=max(MINIMUM_OPENING,min(1.,g.target+direction*rate*.05))
            g.set_target(target);g.cycle()
            sent=round(d.targets[-1]*255)
            if sent!=previous:
                assert abs(sent-previous)>=4
                assert direction*(sent-previous)>0
                assert (sent<=round(target*255) if direction>0 else sent>=round(target*255))
                previous=sent
            if target==endpoint:break
        assert round(d.targets[-1]*255)==opening_raw(endpoint)
        g.hold()


def test_endpoint_reservation_is_not_permission_to_overshoot_or_flush_on_release():
    g,d=ready();g.set_target(19/255);g.cycle()
    for raw in (15,14,13):g.set_target(raw/255);g.cycle()
    assert round(d.targets[-1]*255)==19
    g.hold();g.cycle();assert round(d.targets[-1]*255)==19
    g.set_target(12/255);g.cycle();assert round(d.targets[-1]*255)==12
    assert g.snapshot()['limits']['maximum']==1.


@pytest.mark.parametrize('guarded',[False,True])
@pytest.mark.parametrize('rate',[.025,.05,.1])
def test_real_encoder_accepts_endpoints_and_geometry_commits_identical_raw_counts(guarded,rate):
    from e6_phosphobot.gripper import CanDevice
    from e6_phosphobot.gripper_geometry import request_state
    # Exercise the production CanDevice encoder; replace only the physical bus.
    packets=[]
    class Channel:
        def exchange(self,packet):
            packets.append(packet)
            return dict(position_raw=packet[1],position_percent=packet[1]*100/255,
                        state=0,host_perf_counter_ns=1)
    device=CanDevice.__new__(CanDevice);device.channel=Channel()
    g=Gripper(device_factory=lambda _:device,clock=lambda:1,rate_s=rate)
    checked=[]
    def guard(snapshot,target):
        expected=request_state(snapshot,1)[-1]
        checked.append(expected)
        assert expected==round(target*255)
        return True
    if guarded:g.motion_guard=guard
    g.initialize()
    for _ in range(3):g.cycle()
    from e6_phosphobot.gripper import MINIMUM_OPENING
    for endpoint,direction in ((MINIMUM_OPENING,-1),(1.,1),(MINIMUM_OPENING,-1),(1.,1)):
        for _ in range(1000):
            g.set_target(max(MINIMUM_OPENING,min(1.,g.target+direction*rate*.05)))
            g.cycle()
            assert g.snapshot()['phase']=='ready'
            if g.target==endpoint:break
        for _ in range(40):g.cycle()
        s=g.snapshot()
        assert s['arrived'] and s['target_raw']==s['command_raw']==s['sent_raw']==opening_raw(endpoint)
        assert packets[-1][1]==opening_raw(endpoint)
        before=len(packets);g.hold()
        for _ in range(4):g.cycle()
        assert all(p[1]==opening_raw(endpoint) for p in packets[before:])
    assert all(0<=p[1]<=255 for p in packets)
    if guarded:assert checked

def test_unreached_initialization_is_retained_as_fault():
    g,d=ready();g.phase='initializing';g.arrived_count=0
    g.target=.35;g.clock=lambda:7_000_000_000
    d.exchange=lambda _:dict(position_percent=29,position_raw=74,state=0,host_perf_counter_ns=6_990_000_000)
    g.cycle();assert g.snapshot()['phase']=='fault'


def test_exit_during_can_configuration_cannot_send_a_late_motion():
    d=Device();g=Gripper()
    def configure(_):g.halt.set();return d
    g.factory=configure;g.initialize();g.cycle()
    assert d.targets==[]


def test_close_preserves_inflight_feedback_and_marks_telemetry_unavailable():
    import threading
    entered=threading.Event();finish=threading.Event()
    d=Device();original=d.exchange
    def exchange(target):entered.set();assert finish.wait(1);return original(target)
    d.exchange=exchange
    g=Gripper(device_factory=lambda _:d,clock=lambda:1);g.start();g.initialize()
    assert entered.wait(1)
    g.halt.set();finish.set();g.close()
    assert g.snapshot()['actual'] is None and g.snapshot()['phase']=='closed'
    assert any(r['event']=='gripper_feedback' for r in g.drain())
