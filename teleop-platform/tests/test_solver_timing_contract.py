from copy import deepcopy
import pytest
from e6_phosphobot.solver_timing import configure_timing,cycle_options,validate_timing,verify_timing_reply


def profile():
    return dict(version=1,period_ms=50,callback_budget_ms=55,request_timeout_ms=70,
                retreat_budget_ms=50,geometry_budget_ms=48)


def params():
    return dict(moveit_servo=dict(publish_period=.05),update_period=.05)


def test_pinned_timing_sets_both_native_parameters_and_request_deadline():
    p=params();expected=profile()
    timing=configure_timing(p,dict(solver_timing=expected))
    assert cycle_options(timing)==dict(callback_budget_ms=55,request_timeout_ms=70)
    assert p['moveit_servo']['publish_period']==p['update_period']==.05
    assert p['e6_callback_budget_ms']==55 and p['e6_retreat_budget_ms']==50
    assert p['e6_geometry_budget_ms']==48 and p['e6_request_timeout_ms']==70
    expected['callback_budget_ms']=54
    assert timing['callback_budget_ms']==55
    verify_timing_reply(dict(solver_timing=deepcopy(timing)),timing)


def test_legacy_binary_retains_old_deadlines_without_new_native_parameters():
    p=params();before=deepcopy(p)
    assert configure_timing(p,{}) is None and p==before
    assert cycle_options(None)=={}
    verify_timing_reply({},None)


@pytest.mark.parametrize('key',list(profile()))
def test_missing_or_noninteger_fields_are_rejected(key):
    for value in (None,True,float(profile()[key])):
        t=profile();t[key]=value
        with pytest.raises(ValueError):validate_timing(t)
    t=profile();del t[key]
    with pytest.raises(ValueError):validate_timing(t)


@pytest.mark.parametrize('change',[
    dict(version=2),dict(period_ms=70),dict(geometry_budget_ms=0),
    dict(retreat_budget_ms=55),dict(geometry_budget_ms=51),
    dict(request_timeout_ms=55),dict(request_timeout_ms=101),dict(unused=1),
])
def test_invalid_timebase_or_budget_contract_fails_before_start(change):
    t=profile();t.update(change)
    with pytest.raises(ValueError):configure_timing(params(),dict(solver_timing=t))


@pytest.mark.parametrize('value',[None,{},dict(callback_budget_ms=42),profile()|dict(callback_budget_ms=54)])
def test_missing_or_mismatched_reply_cannot_produce_a_candidate(value):
    with pytest.raises(ValueError):verify_timing_reply(dict(solver_timing=value),profile())


def test_manifestless_reply_and_unpinned_config_override_are_rejected():
    with pytest.raises(ValueError):verify_timing_reply(dict(solver_timing=profile()),None)
    p=params();p['e6_callback_budget_ms']=55
    with pytest.raises(ValueError):configure_timing(p,{})
    p=params();p['update_period']=.07
    with pytest.raises(ValueError):configure_timing(p,dict(solver_timing=profile()))
