from copy import deepcopy

import pytest

from e6_phosphobot.cpp_cycle import CppCycle
from e6_phosphobot.jazzy_host import solver_actual
from test_cpp_cycle import context, response


@pytest.mark.parametrize('change',['fresh','expired','future','disconnect','missing','generation','nan'])
def test_projection_keeps_request_and_rejection_semantics(change):
    now=1_000_000_000
    row=context(now)
    row['actual'].update(age_ms=0.,raw_b64='unused',recording={'frame':'unused'})
    row['actual']['status']['unrelated_device_field']={'x':1}
    if change=='expired':row['actual']['received_ns']=now-100_000_001
    elif change=='future':row['actual']['received_ns']=now+1
    elif change=='disconnect':row['actual']['connected']=False
    elif change=='missing':row['actual']['status']=None
    elif change=='generation':row['actual']['generation']=2
    elif change=='nan':row['actual']['status']['joint_state']['q_actual'][2]=float('nan')
    results=[]
    for projected in (False,True):
        c=CppCycle()
        c.update(context(now-2_000_000,active=False));c.request(now-2_000_000)
        c.complete(response(c),now-1_000_000)
        updated=deepcopy(row)
        if projected:updated['actual']=solver_actual(updated['actual'])
        c.update(updated)
        try:request=c.request(now)
        except ValueError as exc:request=(type(exc).__name__,str(exc))
        results.append((request,c.info,c.rejections))
    assert results[0]==results[1]
    actual=solver_actual(row['actual'])
    assert 'raw_b64' not in actual and 'recording' not in actual
    assert actual['received_ns']==row['actual']['received_ns']
