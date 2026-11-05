"""Feedback-only startup, including first-command geometry and cancellation."""
import pytest
from e6_phosphobot.gripper import Gripper,CanDevice
from e6_phosphobot import omnipicker_can as can

def rig(raw=102):
    now=[1];packets=[];feedback=dict(position_raw=raw,position_percent=raw*100/255,state=0,velocity_raw=0,fault=0)
    class Channel:
        def exchange(self,packet):
            packets.append(packet)
            return dict(feedback,host_perf_counter_ns=now[0])
    d=CanDevice.__new__(CanDevice);d.channel=Channel()
    g=Gripper(device_factory=lambda _:d,clock=lambda:now[0],rate_s=.1)
    return g,now,packets,feedback

def attach(g,now):
    assert g.connect()
    for _ in range(3):g.cycle();now[0]+=50_000_000
    assert g.snapshot()['valid'] and g.snapshot()['arrived']

@pytest.mark.parametrize('raw',[0,1,102,254,255])
def test_attachment_observes_current_position_and_never_sends_35_percent(raw):
    g,now,packets,f=rig(raw);attach(g,now)
    for _ in range(20):g.cycle();now[0]+=50_000_000
    assert all(p==can.FW335_OBSERVATION for p in packets)
    assert g.snapshot()['actual']==raw/255
    assert g.snapshot()['sent_raw'] is None and g.snapshot()['effective_target'] is None
    assert not g.connect()  # refresh/reconnect cannot reinitialize

def test_query_cannot_be_a_zero_position_echo_and_motion_encoder_still_rejects_zero_speed():
    reply=bytes((0,0,0,0,0,25,0,0))
    decoded=can.decode_feedback(can.FRAME.pack(1,8,reply),0,1,can.FW335_OBSERVATION)
    assert decoded['position_raw']==0
    with pytest.raises(ValueError):can.encode_command(40,dict(can.LIMITS,velocity=0),bounds=(0,100))

@pytest.mark.parametrize('state,fault',[(3,1),(0,1),(2,1)])
def test_attachment_faults_are_latched_without_motion_or_retry(state,fault):
    g,now,packets,f=rig();f.update(state=state,fault=fault)
    g.connect();g.cycle();count=len(packets)
    assert g.phase=='fault' and not g.connect()
    for _ in range(10):g.cycle()
    assert len(packets)==count and all(p==can.FW335_OBSERVATION for p in packets)

def test_pending_first_geometry_keeps_observing_and_release_cancels_without_motion():
    g,now,packets,f=rig();attach(g,now);g.motion_guard=lambda *_:False
    for _ in range(12):
        g.set_rate(.08,.05);g.cycle();now[0]+=50_000_000
        assert g.snapshot()['valid']
    assert all(p==can.FW335_OBSERVATION for p in packets)
    g.hold();g.motion_guard=lambda *_:True
    for _ in range(8):g.cycle();now[0]+=50_000_000
    assert all(p==can.FW335_OBSERVATION for p in packets)
    g.set_rate(-.08,.05);g.cycle()
    assert packets[-1][2]>0 and packets[-1][1]<102

def test_feedback_jitter_does_not_create_idle_commands():
    g,now,packets,f=rig();attach(g,now)
    for raw in [101,102,103,102]*5:
        f.update(position_raw=raw,position_percent=raw*100/255)
        g.cycle();now[0]+=50_000_000
    assert all(p==can.FW335_OBSERVATION for p in packets)

def test_moving_or_missing_feedback_cannot_finish_attachment():
    g,now,packets,f=rig();f.update(state=1,velocity_raw=5);g.connect()
    for _ in range(130):g.cycle();now[0]+=50_000_000
    assert g.phase=='fault' and not g.initialized


@pytest.mark.parametrize('raw',[0,11,12,13,102,255])
def test_installed_stroke_never_remaps_geometry_or_native_feedback(raw):
    from e6_phosphobot.gripper_geometry import raw_position
    g,now,packets,f=rig(raw);attach(g,now)
    s=g.snapshot()
    assert s['opening']==pytest.approx(max(0.,(raw-12)/243))
    assert s['actual']==raw/255 and raw_position(s,now[0])==raw
    assert s['stroke']['closed_raw']==12 and s['stroke']['open_raw']==255
    if raw<12:
        g.set_stroke_rate(-.1,.05);g.cycle()
        assert all(p==can.FW335_OBSERVATION for p in packets)


def test_stroke_input_maps_rate_and_closed_endpoint_to_native_units():
    g,now,packets,f=rig(12);attach(g,now)
    for _ in range(20):
        g.set_stroke_rate(-.1,.05);g.cycle();now[0]+=50_000_000
    assert all(p[1]>=12 for p in packets if p[2])
    assert g.snapshot()['opening']==0
    g.set_stroke_rate(.1,.05);g.cycle()
    assert packets[-1][1]>12 and packets[-1][2]>0
    with pytest.raises(ValueError):g.set_stroke_rate(.101,.05)
    with pytest.raises(ValueError):g.set_target(0.)
