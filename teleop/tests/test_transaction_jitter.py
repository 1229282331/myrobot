import pytest
from test_hardware_control import setup,arm,send,tick


@pytest.mark.parametrize('delay_s',[.05013614,.060,.074])
def test_acknowledged_small_jitter_is_recorded_without_stop_or_replay(delay_s):
    c,d,g=setup('live');arm(g,d);original=d.servo
    def delayed(q,deadline):
        identity=original(q,deadline);c.step(delay_s);return identity
    d.servo=delayed;send(g,d,deadman=True);result=tick(c,d,g)
    assert g.armed and not g.faulted and d.servo_count==1 and d.stop_count==0
    assert result['effective_action']['transaction_jitter_tolerated']
    assert g.status()['transaction_budget']['tolerated_jitter_count']==1
    assert g.pending['command_id']==1
    # There is still only one outstanding command, regardless of the jitter.
    send(g,d,deadman=True)
    g.tick(d.state(),{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert d.servo_count==1


def test_hard_deadline_still_stops_a_late_known_ack_without_retransmitting():
    c,d,g=setup('live');arm(g,d);d.late=True;send(g,d,deadman=True);r=tick(c,d,g)
    assert g.faulted and not g.armed and d.servo_count==1 and d.stop_count==1
    assert r['effective_action']['stop_requested_after_acceptance'] and not r['effective_action']['cycle_completed']
    assert g.status()['transaction_budget']['hard_deadline_ms']==75


def test_normal_release_during_soft_jitter_never_sends_candidate():
    c,d,g=setup('live');arm(g,d);original=d.ik
    def released(*args):
        c.step(.06);d.release_pending=True;return original(*args)
    d.ik=released;send(g,d,deadman=True);r=tick(c,d,g)
    assert r['effective_action'] is None and not g.faulted
    assert d.servo_count==d.grip_count==0
