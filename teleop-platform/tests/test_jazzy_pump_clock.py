"""Exercise the real ROS pump with deterministic local IPC/service doubles."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from e6_phosphobot.runtime_timing import RuntimeTiming

import numpy as np
import pytest


def pump_method():
    source=Path(__file__).parents[1]/'src/e6_phosphobot/jazzy_worker.py'
    tree=ast.parse(source.read_text(encoding='utf-8'))
    pumps=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name in ('pump','_pump')]
    # The method closes over main's latest_info. Bind that test result globally
    # while leaving the executable method itself unchanged.
    for pump in pumps:
        pump.body=[ast.Global(names=n.names) if isinstance(n,ast.Nonlocal) else n for n in pump.body]
    module=ast.fix_missing_locations(ast.Module(body=pumps,type_ignores=[]))
    return compile(module,str(source),'exec')


@pytest.mark.parametrize('command_age_ms,expected_active',[(0,True),(249,True),(251,False)])
def test_received_command_is_not_compared_with_pre_read_clock(command_age_ms,expected_active):
    now=[10_000_000_000];published=[];calls=[];outputs=[]
    def receive():
        now[0]+=2_000_000
        return dict(epoch=3,at_ns=now[0],actual=dict(connected=True,received_ns=now[0]-1_000_000,
            status=dict(joint_state=dict(q_actual=[0]*6),tool_vector_actual={})),
            command=dict(epoch=3,received_ns=now[0]-int(command_age_ms*1e6),
                         intent=dict(joint_rad_s=[0]*6)))
    pipe=NS(receive=receive,offer=outputs.append,flush=lambda:None)
    node=NS(incoming=None,service_progress=lambda:None,boot_complete=True,
        last_model_ns=now[0],model_error=dict(position_mm=0,orientation_deg=0),names=['j']*6,
        pause=NS(service_is_ready=lambda:True),feedback=NS(publish=lambda msg:None),
        get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:None)),paused=True,active_epoch=None,
        active_kind=1,future=None,pause_ns=0,published={},timed=None,candidate=None,
        operation=None,operation_epoch=2,call=lambda *a:calls.append(a),publish=published.append,
        status_code=0,trajectory_rejections=0,last_rejection=None,sample_counts={},last_sample=None,
        last_trajectory=None,last_actual_fk_ms=None)
    env=dict(clock=lambda:now[0],pipe=pipe,latest_info={},servo=NS(poll=lambda:None,pid=1),
        kdl=NS(poll=lambda:None,pid=2),JointState=lambda:NS(header=NS()),np=np,
        profile=dict(urdf_sha256='test',tcp={'profile':'flange'}),args=NS(device_period=.05,motion_profile='normal'),
        get_profile=lambda _:NS(description=lambda:{}),timing=RuntimeTiming(lambda:now[0],lambda:0))
    exec(pump_method(),env)
    node._pump=lambda:env['_pump'](node)
    env['pump'](node)
    info=outputs[-1]['info']
    assert info['ready']
    assert info['transition']['active'] is expected_active
    assert info['transition']['command_age_ms']==pytest.approx(command_age_ms)
    assert bool(published) is expected_active
    assert calls==([('pause',False)] if expected_active else [])
