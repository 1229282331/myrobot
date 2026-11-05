import numpy as np
import json
from pathlib import Path
import pytest
from e6_phosphobot.joint_tracking import JointTrackingGuard


def feedback(now, actual=0., target=0.):
    return dict(received_ns=now,status=dict(joint_state=dict(q_actual=[0,0,0,0,actual,0],q_target=[0,0,0,0,target,0])))


def test_held_tracking_error_stops_after_requested_two_degree_half_second_window():
    g=JointTrackingGuard(30,30);reason=None
    for i in range(45):
        now=1_000_000_000+i*50_000_000;target=i*.1
        reason=g.observe(feedback(now,target=target),now)
        if reason:break
    assert reason=='joint_tracking_sustained' and i==31
    assert g.last_detail['joint']==5
    assert g.last_detail['duration_ms']==500


def test_real_v304_cross_reset_jump_cannot_use_a_new_seed_as_permission():
    g=JointTrackingGuard(30,30);old=np.array([0,0,0,0,77.642014,0.])
    g.committed(old,1_000_000_000)
    new=old.copy();new[4]=85.504978
    assert g.candidate(new,feedback(1_072_241_000,actual=85.5,target=85.5),1_072_241_000)=='joint_command_discontinuity'


def test_late_command_does_not_earn_an_unbounded_jump():
    g=JointTrackingGuard(30,30);g.committed(np.zeros(6),1_000_000_000)
    assert g.candidate([0,0,0,0,7,0],feedback(9_000_000_000,actual=7,target=7),9_000_000_000)=='joint_command_discontinuity'


def test_fresh_initial_candidate_cannot_jump_from_current_controller_target():
    g=JointTrackingGuard(30,30);s=feedback(1_000_000_000)
    assert g.candidate([0,0,0,0,.07,0],s,1_000_000_000) is None
    assert g.candidate([0,0,0,0,.2,0],s,1_000_000_000)=='joint_command_discontinuity'


def test_ordinary_lagged_motion_and_neutral_after_stop_are_accepted():
    g=JointTrackingGuard(30,30)
    for i in range(100):
        now=1_000_000_000+i*50_000_000
        actual=max(0,(i-2)*.05);target=max(0,(i-1)*.05)
        s=feedback(now,actual,target)
        assert g.observe(s,now) is None
        q=np.array([0,0,0,0,i*.05,0.])
        assert g.candidate(q,s,now) is None
        g.committed(q,now)
    assert g.rebase(feedback(now+1_000_000_000,actual,target)) is None
    assert g.last_sent is None


def test_reprepare_does_not_mask_residual_controller_target():
    g=JointTrackingGuard(30,30)
    assert g.rebase(feedback(1_000_000_000,85.21,78.43))=='joint_target_not_settled'


@pytest.mark.parametrize('offset',[-2.,-1.473,-1.,-.46,.46,1.,1.473,2.])
def test_stopped_loaded_offset_restarts_from_actual_with_bounded_first_step(offset):
    g=JointTrackingGuard(30,30)
    s=feedback(1_000_000_000,actual=12.+offset,target=12.)
    assert g.rebase(s) is None
    for direction in (-1,1):
        q=[0,0,0,0,12.+offset+direction*.07,0]
        assert g.candidate(q,s,s['received_ns']) is None
    # The relaxed idle gate must not send the former loaded target.
    assert g.candidate([0,0,0,0,12.,0],s,s['received_ns'])=='joint_command_discontinuity'
    assert g.candidate([0,0,0,0,12.+offset+.2,0],s,s['received_ns'])=='joint_command_discontinuity'


@pytest.mark.parametrize('offset',[-2.001,2.001])
def test_stopped_offset_above_two_degrees_keeps_old_writer_history(offset):
    g=JointTrackingGuard(30,30)
    g.committed([0,0,0,0,12.,0],900_000_000)
    assert g.rebase(feedback(1_000_000_000,12.+offset,12.))=='joint_target_not_settled'
    assert g.last_sent[4]==12.
    assert g.last_detail['limit_deg']==2.
    assert g.last_detail['joint']==5


def test_actual_drift_after_prepare_cannot_expand_initial_step():
    g=JointTrackingGuard(30,30)
    assert g.rebase(feedback(1_000_000_000,12.46,12.)) is None
    s=feedback(2_000_000_000,12.76,12.)
    assert g.candidate([0,0,0,0,12.77,0],s,s['received_ns'])=='joint_command_discontinuity'


@pytest.mark.parametrize('offset',[-2.,-1.473,-.46,.46,1.473,2.])
def test_loaded_offset_within_two_degrees_is_tolerated_idle_and_live(offset):
    g=JointTrackingGuard(30,30)
    s=feedback(1_000_000_000,offset,0.)
    assert g.rebase(s) is None
    for i in range(41):
        now=1_000_000_000+i*50_000_000
        assert g.observe(feedback(now,offset,0.),now) is None


def test_repeated_sample_cannot_accumulate_false_no_progress_duration():
    g=JointTrackingGuard(30,30);s=feedback(1_000_000_000,target=.5)
    for _ in range(100):assert g.observe(s,1_050_000_000) is None
    assert len(g.history)==1
    assert g.observe(s,1_200_000_000)=='tracking_feedback_stale'


def test_large_feedback_error_still_stops_at_half_second_without_immediate_two_degree_gate():
    g=JointTrackingGuard(30,30)
    for i in range(10):
        now=1_000_000_000+i*50_000_000
        assert g.observe(feedback(now,85.21,78.43),now) is None
    assert g.observe(feedback(1_500_000_000,85.21,78.43),1_500_000_000)=='joint_tracking_sustained'


@pytest.mark.parametrize('sign',[-1,1])
def test_return_to_two_degrees_resets_continuous_error_duration(sign):
    g=JointTrackingGuard(30,30)
    for ms,error in ((0,2.1),(499,2.),(500,2.1),(999,2.1)):
        now=1_000_000_000+ms*1_000_000
        assert g.observe(feedback(now,target=sign*error),now) is None
    now=2_000_000_000
    assert g.observe(feedback(now,target=sign*2.1),now)=='joint_tracking_sustained'
    assert g.last_detail['duration_ms']==500


def test_following_timer_is_per_joint_not_sum_of_different_joint_excursions():
    g=JointTrackingGuard(30,30)
    for ms,j in ((0,3),(300,4),(500,4),(799,4),(800,4)):
        now=1_000_000_000+ms*1_000_000;s=feedback(now)
        s['status']['joint_state']['q_target'][j]=2.1
        result=g.observe(s,now)
        if ms<800:assert result is None
        else:assert result=='joint_tracking_sustained' and g.last_detail['joint']==5


def test_execution_deviation_timer_also_resets_on_reentry():
    g=JointTrackingGuard(30,30)
    g.committed([0]*6,1_000_000_000)
    # Moving actual/target together isolates execution history from same-frame
    # following error and from the no-progress window.
    for ms,actual in ((0,2.1),(499,2.),(500,2.1),(999,2.2)):
        now=1_000_000_000+ms*1_000_000
        assert g.observe(feedback(now,actual,actual),now) is None
    now=2_000_000_000
    assert g.observe(feedback(now,2.3,2.3),now)=='joint_execution_lag'
    assert g.last_detail['duration_ms']==500


@pytest.mark.parametrize('case_index', [0, 1])
def test_recorded_moving_joint_is_not_stopped_by_unexecuted_command_lead(case_index):
    case = json.loads((Path(__file__).parent/'fixtures/tracking-v310.json').read_text())['cases'][case_index]
    g = JointTrackingGuard(30, 30)
    commands = iter(case['commands'])
    pending = next(commands, None)
    for frame in case['frames']:
        now = frame['at_ns']
        while pending is not None and pending['at_ns'] <= now:
            g.committed(pending['q'], pending['at_ns'])
            pending = next(commands, None)
        sample = dict(received_ns=now, status=dict(joint_state=dict(
            q_actual=frame['actual'], q_target=frame['target'])))
        assert g.observe(sample, now) is None, g.last_detail


def test_controller_tracking_error_is_still_rejected_while_joint_moves():
    g = JointTrackingGuard(30, 30)
    reasons = []
    for i in range(11):
        now = 1_000_000_000 + i*50_000_000
        actual = i*.05
        reasons.append(g.observe(feedback(now, actual, actual+2.1), now))
    assert reasons[:-1] == [None]*10
    assert reasons[-1] == 'joint_tracking_sustained'


def test_stalled_controller_cannot_hide_outstanding_command_by_echoing_actual():
    g = JointTrackingGuard(30, 30)
    g.rebase(feedback(1_000_000_000))
    g.committed([0, 0, 0, 0, 2.1, 0], 1_001_000_000)
    reasons = []
    for i in range(11):
        now = 1_001_000_000+i*50_000_000
        reasons.append(g.observe(feedback(now), now))
    assert reasons[:-1] == [None]*10
    assert reasons[-1] == 'joint_no_progress'


def test_outstanding_command_has_same_two_degree_half_second_allowance():
    g = JointTrackingGuard(30, 30)
    g.committed([0, 0, 0, 0, 2.01, 0], 1_000_000_000)
    for i in range(10):
        now=1_000_000_000+i*50_000_000
        assert g.observe(feedback(now),now) is None
    assert g.observe(feedback(1_500_000_000),1_500_000_000)=='joint_execution_lag'


@pytest.mark.parametrize('direction',[-1,1])
def test_steady_200ms_execution_lag_does_not_trigger_two_degree_new_target_bound(direction):
    g=JointTrackingGuard(30,30)
    g.rebase(feedback(1_000_000_000))
    for i in range(60):
        now=1_000_000_000+i*50_000_000
        # A bounded ramp avoids disguising an initial discontinuity as lag.
        command=direction*.025*i*i if i<10 else direction*(2.5+(i-10)*.5)
        old=max(0,i-4);old_target=max(0,i-3)
        def q(k):return direction*.025*k*k if k<10 else direction*(2.5+(k-10)*.5)
        s=feedback(now,q(old),q(old_target))
        assert g.observe(s,now) is None,g.last_detail
        assert g.candidate([0,0,0,0,command,0],s,now) is None,g.last_detail
        g.committed([0,0,0,0,command,0],now)


def test_slow_progress_cannot_hide_unbounded_execution_lag():
    g=JointTrackingGuard(30,30);g.rebase(feedback(1_000_000_000))
    reason=None
    for i in range(50):
        now=1_000_000_000+i*50_000_000
        g.committed([0,0,0,0,i*.5,0],now)
        reason=g.observe(feedback(now,i*.05,i*.05),now)
        if reason:break
    assert reason=='joint_execution_lag' and i<25


@pytest.mark.parametrize('direction', [-1, 1])
def test_continuous_motion_with_servoj_lead_in_both_directions(direction):
    g = JointTrackingGuard(30, 30)
    for i in range(40):
        now = 1_000_000_000+i*50_000_000
        actual = direction*i*.3
        g.committed([0, 0, 0, 0, actual+direction*.9, 0], now-40_000_000)
        assert g.observe(feedback(now, actual, actual+direction*.24), now) is None


@pytest.mark.parametrize('case_index', range(5))
def test_recorded_continuous_commands_are_not_compared_to_delayed_target(case_index):
    case = json.loads((Path(__file__).parent/'fixtures/tracking-v311.json').read_text())['cases'][case_index]
    g = JointTrackingGuard(30, 30)
    g.committed(case['last_sent_deg'], case['last_sent_ns'])
    assert g.observe(case['sample'], case['at_ns']) is None
    assert g.candidate(case['candidate_deg'], case['sample'], case['at_ns']) is None, g.last_detail


@pytest.mark.parametrize('direction', [-1, 1])
def test_true_command_jump_rejected_even_when_controller_target_equals_candidate(direction):
    g = JointTrackingGuard(30, 30)
    g.committed([0]*6, 1_000_000_000)
    now = 1_025_000_000
    q = direction*.9
    assert g.candidate([0, 0, 0, 0, q, 0], feedback(now, q, q), now) == 'joint_command_discontinuity'


@pytest.mark.parametrize('case_index',range(5))
def test_v314_original_raw_window_passes_with_bounded_written_history(case_index):
    case=json.loads((Path(__file__).parent/'fixtures/tracking-v316.json').read_text())['cases'][case_index]
    g=JointTrackingGuard(30,30);commands=iter(case['commands']);pending=next(commands,None)
    for frame in case['frames']:
        now=frame['received_ns']
        while pending is not None and pending['at_ns']<=now:
            g.committed(pending['q'],pending['at_ns']);pending=next(commands,None)
        assert g.observe(frame,now) is None,g.last_detail
    while pending is not None:
        g.committed(pending['q'],pending['at_ns']);pending=next(commands,None)
    assert g.observe(case['sample'],case['at_ns']) is None,g.last_detail
    assert g.candidate(case['candidate_deg'],case['sample'],case['at_ns']) is None,g.last_detail


def test_bounded_delay_reversal_and_pause_do_not_look_like_stall():
    g=JointTrackingGuard(30,30);g.rebase(feedback(1_000_000_000))
    commands=[0.]*4+[.01*i*i for i in range(10)]+[1.+.2*i for i in range(10)]
    commands+=list(reversed(commands))+[0.]*10
    for i,q in enumerate(commands):
        now=1_000_000_000+i*50_000_000
        actual=commands[max(0,i-4)];target=commands[max(0,i-3)]
        s=feedback(now,actual,target)
        assert g.observe(s,now) is None,g.last_detail
        assert g.candidate([0,0,0,0,q,0],s,now) is None,g.last_detail
        g.committed([0,0,0,0,q,0],now)


@pytest.mark.parametrize('value', [
    [float('nan')]*6, [float('inf')]*6, [[1]]*6,
    np.ones((6, 1)), {i: i for i in range(6)}, set(range(6)), [0]*5,
])
def test_joint_vectors_reject_nonfinite_nested_or_unordered_values(value):
    with pytest.raises(ValueError, match='Invalid joint vector'):
        JointTrackingGuard.vector(value)


def test_written_history_owns_its_joint_values_after_caller_mutation():
    g = JointTrackingGuard(30, 30)
    q = np.zeros(6)
    g.committed(q, 1_000_000_000)
    q[4] = 9
    assert list(g.last_sent) == [0]*6
    assert list(g.sent_history[0][1]) == [0]*6
    assert g.candidate(q, feedback(1_050_000_000), 1_050_000_000) == 'joint_command_discontinuity'


def test_feedback_history_does_not_alias_mutable_sdk_lists():
    g = JointTrackingGuard(30, 30)
    s = feedback(1_000_000_000, actual=.25, target=.5)
    assert g.observe(s, 1_000_000_000) is None
    s['status']['joint_state']['q_actual'][4] = 12
    s['status']['joint_state']['q_target'][4] = 12
    assert g.history[0][1][4] == .25
    assert g.history[0][2][4] == .25
