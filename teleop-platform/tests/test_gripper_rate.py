"""Continuous-input behavior at the real encoder and geometry boundary."""
import math
import pytest
from e6_phosphobot.gripper import Gripper,CanDevice,opening_raw
from e6_phosphobot.gripper_geometry import request_state

def rig():
    now=[1];packets=[];actual=[89.];allow=[True];checks=[]
    class Channel:
        def exchange(self,p):
            packets.append(p)
            # Independent bounded-speed actuator, including fractional state.
            delta=p[1]-actual[0];actual[0]+=max(-p[2]*2*.05,min(p[2]*2*.05,delta))
            raw=round(actual[0])
            return dict(position_raw=raw,position_percent=raw*100/255,state=int(abs(delta)>1),
                        velocity_raw=p[2],host_perf_counter_ns=now[0])
    d=CanDevice.__new__(CanDevice);d.channel=Channel()
    g=Gripper(device_factory=lambda _:d,clock=lambda:now[0],rate_s=.1)
    def guard(s,t):
        checks.append(t);assert request_state(s,now[0])[-1]==round(t*255)
        return allow[0]
    g.motion_guard=guard;g.initialize()
    for _ in range(4):g.cycle();now[0]+=50_000_000
    assert g.snapshot()['valid']
    return g,now,packets,actual,allow,checks

@pytest.mark.parametrize('rate',[.01,.025,.05,.08,.1,-.01,-.025,-.05,-.08,-.1])
def test_live_rate_progresses_and_keeps_bounded_feedback_lead(rate):
    g,now,packets,actual,allow,checks=rig();start=actual[0]
    for _ in range(45):
        before=round(actual[0]);g.set_rate(rate,.05);g.cycle();now[0]+=50_000_000
        assert abs(packets[-1][1]-before)<=8
        assert 0<=packets[-1][1]<=255
        assert packets[-1][3:]==bytes([255,25,25,0,0])
    assert math.copysign(1,rate)*(actual[0]-start)>2
    g.hold();g.cycle();held=g.snapshot()['sent_raw'];count=len(packets)
    for _ in range(30):g.cycle();now[0]+=50_000_000
    assert all(p[1]==held for p in packets[count:])
    assert checks

def test_small_input_starts_without_waiting_for_four_count_integral():
    g,now,packets,*_=rig();g.set_rate(.025,.05);g.cycle()
    assert packets[-1][1]>89 and packets[-1][2]<25

def test_start_and_reverse_ramp_native_speed_without_delaying_release():
    g,now,packets,*_=rig();vel=[]
    for _ in range(12):
        g.set_rate(.08,.05);g.cycle();now[0]+=50_000_000;vel.append(g.velocity)
    assert vel[0]==1 and vel[-1]==11
    assert vel==sorted(vel) and max(b-a for a,b in zip(vel,vel[1:]))<=2
    before=g.last_sent;g.set_rate(-.08,.05)
    assert g.velocity==1 and g.command_target<before
    g.cycle();g.hold();assert not g.rate_active

def test_reducing_speed_cannot_make_the_command_faster():
    g,now,packets,*_=rig()
    for _ in range(12):g.set_rate(.08,.05);g.cycle();now[0]+=50_000_000
    old=g.velocity
    for _ in range(12):
        g.set_rate(.02,.05);g.cycle();now[0]+=50_000_000
        assert g.velocity<=old;old=g.velocity
    assert g.velocity==3

def test_blocked_direction_does_not_accumulate_and_reversal_discards_pending():
    g,now,packets,actual,allow,checks=rig();allow[0]=False
    for _ in range(100):g.set_rate(.1,.05);g.cycle();now[0]+=50_000_000
    assert packets[-1][1]==89
    assert abs(g.target*255-actual[0])<=8.00001
    allow[0]=True;g.set_rate(-.1,.05);g.cycle()
    assert packets[-1][1]<89

def test_pending_geometry_target_remains_stable_until_can_commits():
    g,now,packets,actual,allow,checks=rig();allow[0]=False
    for _ in range(10):g.set_rate(.1,.05);g.cycle();now[0]+=50_000_000
    assert len(set(checks))==1
    allow[0]=True;g.cycle();assert packets[-1][1]==round(checks[-1]*255)

def test_lost_rate_updates_drop_unsent_intent_without_a_new_command():
    g,now,packets,*_=rig();g.set_rate(.1,.05)
    now[0]+=251_000_000;g.cycle()
    assert packets[-1][1]==89 and not g.rate_active

def test_both_endpoints_and_reverse_from_endpoint():
    g,now,packets,*_=rig()
    for rate,endpoint in ((.1,1.),(-.1,0.),(.1,1.)):
        for _ in range(260):g.set_rate(rate,.05);g.cycle();now[0]+=50_000_000
        assert packets[-1][1]==opening_raw(endpoint)

def test_stale_feedback_and_invalid_rate_cannot_select_motion():
    g,now,packets,*_=rig();now[0]+=201_000_000
    assert not g.set_rate(.1,.05)
    for rate,dt in ((float('nan'),.05),(.11,.05),(.1,.5),(.1,-.1)):
        with pytest.raises(ValueError):g.set_rate(rate,dt)

def test_release_selects_one_bounded_near_feedback_target():
    g,now,packets,actual,*_=rig();g.set_rate(.01,.05);g.cycle()
    before=actual[0];target=packets[-1][1]
    g.hold()
    for _ in range(40):g.cycle();now[0]+=50_000_000
    assert abs(actual[0]-before)<=1.5 and packets[-1][1]<target and packets[-1][2]>0
    g.set_rate(-.01,.05);g.cycle()
    assert packets[-1][2]>0 and packets[-1][1]<target

def test_braking_target_is_checked_and_neutral_never_chases_feedback():
    g,now,packets,actual,allow,checks=rig();g.set_rate(.01,.05);g.cycle()
    old=packets[-1][1];allow[0]=False;g.hold();desired=g.command_target
    for _ in range(4):g.hold();g.cycle();now[0]+=50_000_000
    assert packets[-1][1]==old and g.command_target==desired
    allow[0]=True;g.cycle()
    assert packets[-1][1]==round(desired*255) and checks[-1]==desired

@pytest.mark.parametrize('rate',[.01,-.01])
def test_delayed_brake_permission_cannot_reverse_after_release(rate):
    g,now,packets,actual,allow,checks=rig();g.set_rate(rate,.05);g.cycle()
    sent=packets[-1][1];allow[0]=False;g.hold()
    # An asynchronously delayed result can arrive after the actuator passed
    # the proposed stopping point. It must never command a correction back.
    for _ in range(60):g.hold();g.cycle();now[0]+=50_000_000
    stopped=actual[0];allow[0]=True
    for _ in range(10):g.cycle();now[0]+=50_000_000
    assert packets[-1][1]==sent and actual[0]==stopped

@pytest.mark.parametrize('rate',[.0025,-.0025])
def test_below_native_velocity_resolution_still_bounds_position_lead(rate):
    g,now,packets,actual,*_=rig();origin=actual[0]
    # Position tracking at sub-byte speeds cannot be promised continuous;
    # native minimum speed must still not produce unbounded extra travel.
    for step in range(400):
        g.set_rate(rate,.05);g.cycle();now[0]+=50_000_000
        reference=origin+rate*.05*(step+1)*255
        assert abs(actual[0]-reference)<=4.5
        assert packets[-1][2]==1
    g.hold();g.cycle()
    assert not g.rate_active
