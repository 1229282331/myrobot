import math
import numpy as np
import pytest
from e6_phosphobot.timed_trajectory import TimedTrajectory,parse_provenance


def trajectory():
    times=[1_000_000_000,1_050_000_000,1_100_000_000]
    velocity=np.array([.001,-.002,.003,-.004,.005,-.006])
    base=np.array([.1,-.2,.3,-.4,.5,-.6])
    q=[base+velocity*i*.05 for i in range(3)]
    trace='e6_trace_v1;1000000000,j,900000000;1050000000,j,950000000;1100000000,j,1000000000;1150000000,j,1050000000'
    return times,q,[velocity]*3,trace,base,velocity


def test_timed_sampling_preserves_constant_velocity_and_exact_input_brackets():
    t,q,v,trace,base,velocity=trajectory();curve=TimedTrajectory(t,q,v,trace)
    result=curve.sample(1_075_000_000)
    np.testing.assert_allclose(result['joints_rad'],base+velocity*.075,atol=1e-14)
    assert [s.input_ns for s in result['bracket_sources']]==[950_000_000,1_000_000_000]
    assert result['window_sources'][-1].point_ns==1_150_000_000


def test_old_or_unavailable_future_never_extrapolates_or_replays():
    t,q,v,trace,*_=trajectory();curve=TimedTrajectory(t,q,v,trace)
    assert curve.sample(t[0]-1) is None
    assert curve.sample(t[-1]+1) is None
    np.testing.assert_allclose(curve.sample(t[-1])['joints_rad'],q[-1])


def test_immediate_endpoint_uses_complete_point_not_omitted_future():
    t,q,v,trace,*_=trajectory();curve=TimedTrajectory(t,q,v,trace)
    result=curve.latest_complete()
    np.testing.assert_array_equal(result['joints_rad'],q[-1])
    assert result['sample_time_ns']==t[-1]
    assert result['bracket_sources'][-1].point_ns==t[-1]
    assert result['window_sources'][-1].point_ns>result['sample_time_ns']
    assert result['bracket_sources'][-1].input_ns==1_000_000_000


@pytest.mark.parametrize('change',['nan','reorder','duplicate','wrong_source_time','missing_future','wrong_shape'])
def test_malformed_or_untraceable_trajectory_is_rejected(change):
    t,q,v,trace,*_=trajectory()
    if change=='nan':q[1][4]=math.nan
    if change=='reorder':t[0],t[1]=t[1],t[0]
    if change=='duplicate':t[1]=t[0]
    if change=='wrong_source_time':trace=trace.replace('1050000000,j,950000000','1050000001,j,950000000')
    if change=='missing_future':trace=trace.rsplit(';',1)[0]
    if change=='wrong_shape':q[0]=q[0][:-1]
    with pytest.raises(ValueError):TimedTrajectory(t,q,v,trace)


def test_state_seed_and_halt_sources_remain_distinct_from_motion_inputs():
    sources=parse_provenance('e6_trace_v1;100,s,0;200,h,50;300,j,70')
    assert [s.kind for s in sources]==['s','h','j']
    with pytest.raises(ValueError):parse_provenance('e6_trace_v1;100,j,0;200,s,0')
