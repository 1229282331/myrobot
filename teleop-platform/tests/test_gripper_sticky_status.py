"""FW335 can retain state=2 while the encoder proves a successful reverse."""
import pytest
import json
from pathlib import Path
from test_gripper_contact import attached,contact
from e6_phosphobot import omnipicker_can as can


def test_recorded_reverse_progress_does_not_reclassify_opening_as_blocked():
    g,d,t=contact();d.raw=208;d.state=2;d.escape_stalled=True;g.cycle()
    g.set_rate(.05,.05);g.cycle();sent=len(d.sent)
    assert d.sent[-1]==(212,1)
    start=t[0]
    # Real packet values from the first reverse in the v288 capture. State
    # stays 2 all the way to target; it never emits the state=1 our old fake did.
    for ms,raw,speed in ((255,209,1),(509,209,0),(759,210,1)):
        t[0]=start+ms*1_000_000;d.raw=raw;d.state=2
        g.rate_received_ns=t[0];g.cycle()
    assert g.snapshot()['contact'] is None
    assert len(d.sent)==sent
    assert g.snapshot()['reported_state']==2


def test_complete_real_reverse_packets_stay_usable_with_sticky_state():
    data=json.loads((Path(__file__).parent/'fixtures/gripper_sticky_state2_v288.json').read_text())
    g,d,t=attached();d.raw=208;d.state=2;d.escape_stalled=True
    g.set_target(200/255);g.cycle();assert g.snapshot()['contact']['direction']==-1
    g.set_rate(.05,.05);g.cycle();start=t[0];released=[]
    d.exchange=lambda *a,**k:d.feedback()
    for row in data['rows']:
        t[0]=start+row['dt_ns'];d.fault,d.state,d.raw=row['bytes'][:3]
        decoded=can.decode_feedback(can.FRAME.pack(1,8,bytes(row['bytes'])),0,1,bytes(data['tx']),allow_stalled=True)
        decoded['host_perf_counter_ns']=t[0]
        d.feedback=lambda decoded=decoded:dict(decoded)
        g.cycle()
        if g.snapshot()['contact'] is None:released.append(row['dt_ns']/1e6)
    assert released and 500<released[0]<1500
    assert g.snapshot()['contact'] is None and g.snapshot()['arrived']
    assert g.snapshot()['reported_state']==2 and g.snapshot()['valid']


def recovered():
    g,d,t=contact();d.escape_stalled=True;g.set_rate(.05,.05);g.cycle()
    t[0]+=800_000_000;d.raw=103;g.rate_received_ns=t[0];g.cycle()
    assert g.snapshot()['contact'] is None
    return g,d,t


def test_sticky_status_with_continuing_progress_does_not_relatch():
    g,d,t=recovered()
    for _ in range(20):
        t[0]+=100_000_000;g.set_rate(.05,.05);d.raw+=1;g.cycle()
        assert g.snapshot()['contact'] is None
    assert d.sent[-1][0]>105


def test_sticky_status_at_measured_target_can_be_held_not_grasp_success():
    g,d,t=recovered();g.hold();d.raw=round(g.last_sent*255)
    for _ in range(5):t[0]+=50_000_000;g.cycle()
    s=g.snapshot();assert s['contact'] is None and s['arrived']
    assert s['reported_state']==2 and 'grasp_success' not in s


def test_new_stall_after_recovery_freezes_target_and_continues_observing():
    g,d,t=recovered()
    for _ in range(60):t[0]+=50_000_000;g.set_rate(.05,.05);g.cycle()
    assert g.snapshot()['contact']['direction']==1
    sent=list(d.sent)
    for _ in range(20):t[0]+=50_000_000;g.set_rate(.05,.05);g.cycle()
    assert d.sent==sent and g.snapshot()['valid']


def test_one_count_jitter_cannot_prove_escape():
    g,d,t=contact();d.escape_stalled=True;g.set_rate(.05,.05);g.cycle()
    for n in range(40):
        t[0]+=50_000_000;d.raw=101+n%2;g.set_rate(.05,.05);g.cycle()
    assert g.snapshot()['contact']['direction']==1
    assert len(d.sent)==2


def test_stop_during_pending_reverse_sends_no_followup_target():
    g,d,t=contact();d.escape_stalled=True;g.set_rate(.05,.05);g.cycle();g.hold();sent=list(d.sent)
    t[0]+=800_000_000;d.raw=103;g.cycle()
    assert g.snapshot()['contact'] is None
    for _ in range(5):t[0]+=50_000_000;g.cycle()
    assert all(x[0]==sent[-1][0] for x in d.sent[len(sent):])


@pytest.mark.parametrize('fault,state',[(1,2),(1,3)])
def test_recovered_sticky_status_never_masks_device_fault(fault,state):
    g,d,t=recovered();d.fault=fault;d.state=state;d.exchange=lambda *a,**k:d.feedback();g.cycle()
    assert g.snapshot()['phase']=='fault'
