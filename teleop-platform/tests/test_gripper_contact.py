"""Contact is not a device fault; no hidden retries or old-target closing step."""
import pytest
from e6_phosphobot.gripper import Gripper
from e6_phosphobot import omnipicker_can as can
from e6_phosphobot.gripper_geometry import request_state,raw_position

class Device:
    def __init__(self,clock):
        self.clock=clock;self.raw=125;self.state=0;self.fault=0;self.sent=[];self.observed=0
        self.escape_stalled=False;self.error=None
    def feedback(self):
        if self.error:raise self.error
        return dict(position_raw=self.raw,position_percent=self.raw*100/255,state=self.state,
                    fault=self.fault,velocity_raw=0,host_perf_counter_ns=self.clock())
    def observe(self):self.observed+=1;return self.feedback()
    def exchange(self,target,*,velocity=None):
        raw=round(target*255);self.sent.append((raw,velocity))
        if raw<=101:self.raw=101;self.state=2
        elif self.escape_stalled:self.state=2
        else:self.raw=raw;self.state=0
        return self.feedback()

def attached(*,stalled=False):
    now=[1];d=Device(lambda:now[0]);d.state=2 if stalled else 0
    g=Gripper(device_factory=lambda _:d,clock=lambda:now[0],rate_s=.1)
    g.connect()
    for _ in range(3):g.cycle();now[0]+=50_000_000
    assert g.snapshot()['phase']=='ready'
    return g,d,now

def contact():
    g,d,t=attached();g.set_target(93/255);g.cycle();t[0]+=50_000_000
    assert g.snapshot()['position_status']=='contact'
    return g,d,t

def test_real_incident_packet_is_feedback_only_when_explicitly_allowed():
    command=bytes((0,93,6,64,25,25,0,0))
    packet=can.FRAME.pack(1,8,bytes.fromhex('0002650030190000'))
    with pytest.raises(can.CheckFailed):can.decode_feedback(packet,0,1,command)
    f=can.decode_feedback(packet,0,1,command,allow_stalled=True)
    assert f['state']==2 and f['fault']==0 and f['position_raw']==101
    for fault,state in ((1,2),(2,2),(3,2),(4,2),(0,3)):
        b=bytes((fault,state,101,0,48,25,0,0))
        with pytest.raises(can.CheckFailed):can.decode_feedback(can.FRAME.pack(1,8,b),0,1,command,allow_stalled=True)

def test_closing_contact_stays_fresh_without_new_position_or_force_commands():
    g,d,t=contact();original=list(d.sent)
    for _ in range(40):
        g.set_rate(-.1,.05);g.cycle();t[0]+=50_000_000
    g.hold();g.cycle()
    s=g.snapshot()
    assert d.sent==original and d.observed>=40
    assert s['valid'] and s['actual']==101/255 and s['phase']=='ready'
    assert not s['arrived'] and s['target_raw']==s['sent_raw']==93
    assert s['contact']['direction']==-1 and s['reported_state']==2
    assert raw_position(s,t[0])==101

def test_reverse_rebases_to_actual_and_matches_geometry_not_old_target():
    g,d,t=contact();checks=[]
    def guard(snapshot,target):
        checks.append(request_state(snapshot,t[0]));assert checks[-1][-1]==round(target*255)
        return True
    g.motion_guard=guard;g.set_rate(.05,.05);g.cycle()
    assert d.sent[-1]==(105,1) # never 97: that would close further from actual101
    assert checks[-1]==[101,93,-1,105]
    assert g.snapshot()['contact'] is None and g.snapshot()['phase']=='ready'

def test_denied_escape_and_release_do_not_repeat_or_flush_a_position_command():
    g,d,t=contact();g.motion_guard=lambda *_:False;original=list(d.sent)
    g.set_rate(.05,.05);g.cycle();assert d.sent==original
    g.hold();g.motion_guard=lambda *_:True;g.cycle();assert d.sent==original
    assert not g.snapshot()['contact']['escape_requested']

def test_expired_escape_is_discarded_before_any_can_motion():
    g,d,t=contact();original=list(d.sent)
    g.set_rate(.05,.05);t[0]+=251_000_000;g.cycle()
    assert d.sent==original and not g.snapshot()['contact']['escape_requested']

def test_delayed_state_clear_after_reverse_is_observed_without_resending():
    g,d,t=contact();d.escape_stalled=True;g.set_rate(.05,.05);g.cycle();n=len(d.sent)
    t[0]+=50_000_000;d.state=1;d.raw=102;g.cycle()
    assert len(d.sent)==n and g.snapshot()['contact'] is None

def test_contact_is_not_target_arrival_or_successful_grasp():
    g,d,t=attached(stalled=True)
    assert not g.snapshot()['arrived']
    assert 'grasp_success' not in g.snapshot()

def test_reverse_cancelled_by_return_to_blocked_direction_before_commit():
    g,d,t=contact();original=list(d.sent)
    g.set_rate(.05,.05);g.set_rate(-.05,.05);g.cycle()
    assert d.sent==original

def test_connection_at_obstruction_observes_without_initialization_motion():
    g,d,t=attached(stalled=True);s=g.snapshot()
    assert s['position_status']=='obstructed' and s['contact']['direction']==0 and not d.sent
    g.set_rate(-.05,.05);g.cycle();assert not d.sent
    g.set_rate(.05,.05);g.cycle();assert d.sent[-1]==(129,1)

def test_lost_contact_does_not_restart_closing_from_held_input():
    g,d,t=contact();d.state=0;d.raw=93;original=list(d.sent)
    for _ in range(10):g.set_rate(-.05,.05);g.cycle();t[0]+=50_000_000
    assert d.sent==original and g.snapshot()['contact'] is not None
    g.set_rate(.05,.05);g.cycle();assert d.sent[-1][0]==97

def test_stalled_escape_sends_once_then_observes_and_changes_blocked_direction():
    g,d,t=contact();d.escape_stalled=True;g.set_rate(.05,.05);g.cycle();n=len(d.sent)
    for _ in range(40):t[0]+=50_000_000;g.set_rate(.05,.05);g.cycle()
    assert len(d.sent)==n and g.snapshot()['contact']['direction']==1
    assert g.snapshot()['position_status']=='obstructed'

def test_true_fault_and_timeout_stay_latched_without_retries():
    for error in (RuntimeError('CAN deadline'),None):
        g,d,t=contact();d.error=error;d.fault=0 if error else 1;g.cycle()
        n=d.observed;g.cycle();assert d.observed==n
        assert g.snapshot()['phase']=='fault' and not g.snapshot()['valid']

def test_stale_contact_feedback_cannot_authorize_opening():
    g,d,t=contact();t[0]+=250_000_000;original=list(d.sent)
    assert not g.set_rate(.05,.05) and not g.set_target(.6)
    assert not g.snapshot()['valid'] and d.sent==original

def test_contact_during_explicit_initialization_remains_failed_initialization():
    now=[1];d=Device(lambda:now[0]);g=Gripper(device_factory=lambda _:d,clock=lambda:now[0])
    g.initialize();g.cycle();assert g.snapshot()['phase']=='fault'
