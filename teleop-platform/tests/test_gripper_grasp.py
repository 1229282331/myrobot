"""Production CAN current parity and task-drop recovery, without hardware."""
import json
from pathlib import Path
import pytest
from e6_phosphobot import omnipicker_can as can
from test_gripper_contact import contact,attached
from test_gripper_attachment import rig,attach


def dropped():
    g,d,t=contact();d.state=3;d.raw=60;g.cycle()
    assert g.snapshot()['position_status']=='dropped'
    return g,d,t


def test_native_current_parity_does_not_change_speed_ramp_or_probe_defaults():
    g,t,packets,f=rig();attach(g,t)
    assert all(p==can.FW335_OBSERVATION for p in packets)
    g.set_rate(.08,.05);g.cycle()
    assert packets[-1][2:6]==bytes([1,255,25,25])
    limits=g.snapshot()['limits']
    assert limits['configured_current_limit_a']==3.0
    assert limits['velocity']==25 and limits['rate_acceleration_s2']==.3
    assert can.LIMITS['force']==64
    with pytest.raises(ValueError):can.encode_command(30,can.FW335_STUDIO_LIMITS)
    assert can.encode_command(30,can.FW335_STUDIO_LIMITS,profile='fw335_studio_3a')[3]==255
    for key in ('velocity','force','acceleration','deceleration'):
        with pytest.raises(ValueError):can.encode_command(30,dict(can.FW335_STUDIO_LIMITS,**{key:256}),profile='fw335_studio_3a')


def test_real_drop_packet_requires_explicit_production_policy_and_never_masks_fault():
    p=bytes.fromhex('00033c232d190000')
    with pytest.raises(can.CheckFailed):can.decode_feedback(can.FRAME.pack(1,8,p),0,1,can.FW335_OBSERVATION,allow_stalled=True)
    f=can.decode_feedback(can.FRAME.pack(1,8,p),0,1,can.FW335_OBSERVATION,allow_stalled=True,allow_dropped=True)
    assert f['state']==3 and f['fault']==0 and f['position_raw']==60
    for fault in (1,2,3,4):
        with pytest.raises(can.CheckFailed):
            can.decode_feedback(can.FRAME.pack(1,8,bytes([fault])+p[1:]),0,1,can.FW335_OBSERVATION,allow_stalled=True,allow_dropped=True)


def test_drop_keeps_feedback_live_and_never_retries_held_input_or_lost_target():
    g,d,t=dropped();sent=list(d.sent)
    for n in range(40):
        t[0]+=50_000_000;g.set_rate(-.1,.05);g.cycle()
        assert g.snapshot()['valid'] and g.snapshot()['phase']=='ready'
    assert d.sent==sent and g.snapshot()['contact']['neutral_seen'] is False
    assert not g.set_target(.6) and not g.set_rate(.1,.05)
    assert len([e for e in g.drain() if e['event']=='gripper_object_dropped'])==1


def test_release_then_open_is_bounded_geometry_checked_and_does_not_regrasp():
    g,d,t=dropped();sent=list(d.sent);g.hold()
    g.set_rate(-.1,.05);g.cycle();assert d.sent==sent
    checks=[]
    g.motion_guard=lambda s,target:checks.append((s['actual'],target)) or False
    g.set_rate(.1,.05);g.cycle();assert d.sent==sent
    assert checks[-1]==(60/255,64/255)
    g.hold();g.motion_guard=lambda *_:True;g.cycle();assert d.sent==sent
    # Simulate new opening; sticky drop status alone must not report recovery.
    d.exchange=lambda target,**kw:d.sent.append((round(target*255),kw.get('velocity'))) or d.feedback()
    g.set_rate(.1,.05);g.cycle();assert d.sent[-1]==(64,1)
    for i in range(5):t[0]+=50_000_000;g.cycle()
    assert g.snapshot()['contact'] is not None
    t[0]+=50_000_000;d.raw=62;g.cycle()
    assert g.snapshot()['contact'] is None and g.snapshot()['reported_state']==3
    assert g.snapshot()['valid']


def test_expired_pending_drop_recovery_never_sends():
    g,d,t=dropped();g.hold();g.set_rate(.1,.05);sent=list(d.sent)
    t[0]+=251_000_000;g.cycle();assert d.sent==sent
    assert not g.snapshot()['contact']['escape_requested']


def test_attachment_at_drop_is_observation_only_until_release_and_open():
    g,t,packets,f=rig(60);f['state']=3;g.connect()
    for _ in range(5):g.cycle();t[0]+=50_000_000
    assert g.snapshot()['valid'] and g.snapshot()['position_status']=='dropped'
    assert all(p==can.FW335_OBSERVATION for p in packets)
    assert not g.set_rate(.1,.05)
    g.hold();g.set_rate(-.1,.05);g.cycle()
    assert all(p==can.FW335_OBSERVATION for p in packets)


@pytest.mark.parametrize('error',[RuntimeError('CAN timeout'),RuntimeError('Invalid frame')])
def test_real_transport_failure_after_drop_still_latches(error):
    g,d,t=dropped();d.error=error;g.cycle();count=d.observed;g.cycle()
    assert g.snapshot()['phase']=='fault' and d.observed==count


def test_new_drop_after_confirmed_reverse_cancels_pending_motion():
    g,d,t=contact();d.escape_stalled=True;g.set_rate(.05,.05);g.cycle()
    d.raw+=2;t[0]+=100_000_000;g.cycle();assert g.snapshot()['contact'] is None
    d.state=3;d.exchange=lambda *a,**k:d.feedback();g.cycle()
    assert g.snapshot()['position_status']=='dropped'
    assert not g.snapshot()['contact']['neutral_seen']


def test_preserved_contact_to_drop_trace_never_generates_a_followup_motion():
    rows=json.loads((Path(__file__).parent/'fixtures/gripper_drop_v289.json').read_text())
    g,d,t=attached();d.raw=64;d.state=2
    d.exchange=lambda target,**kw:d.sent.append((round(target*255),kw.get('velocity'))) or d.feedback()
    g.set_target(56/255);g.cycle();sent=list(d.sent)
    for row in rows:
        t[0]=row['dt_ns']+1_000_000_000
        f=can.decode_feedback(can.FRAME.pack(1,8,bytes(row['bytes'])),0,1,can.FW335_OBSERVATION,allow_stalled=True,allow_dropped=True)
        f['host_perf_counter_ns']=t[0];d.feedback=lambda f=f:dict(f)
        g.hold();g.cycle()
    assert g.snapshot()['position_status']=='dropped' and g.snapshot()['valid']
    assert d.sent==sent and not g.snapshot()['arrived']
