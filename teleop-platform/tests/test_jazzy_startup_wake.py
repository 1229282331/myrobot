"""Run the actual ROS callbacks without ROS; wakes never bypass pump checks."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


def method(name, env):
    source = Path(__file__).parents[1] / 'src/e6_phosphobot/jazzy_worker.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(source), 'exec'), env)
    return env[name]


def test_service_completion_wakes_executor_without_reentrant_pump_or_early_result_use():
    calls = []; callbacks = []; wakes = []
    future = NS(add_done_callback=callbacks.append)
    client = NS(service_is_ready=lambda: True, call_async=lambda req: calls.append(req) or future)
    node = NS(future=None, pause=client, switch=client, epoch=8,
              wake=NS(trigger=lambda: wakes.append('wake')))
    call = method('call', dict(SetBool=NS(Request=NS), ServoCommandType=NS(Request=NS),
                              time=NS(monotonic=lambda: 2.)))
    call(node, 'pause', False)
    assert node.future is future and node.operation == ('pause', False)
    assert node.operation_epoch == 8 and not wakes
    callbacks[0](future)
    assert wakes == ['wake'] and node.future is future
    # Future result and current epoch are still handled by the normal pump.
    call(node, 'pause', True)
    assert len(calls) == 1


@pytest.mark.parametrize('case,expected', [
    ('first', 1), ('later', 0), ('paused', 0), ('old_epoch', 0),
    ('pending_service', 0), ('old_source', 0), ('unknown_source', 0)])
def test_only_current_valid_first_trajectory_requests_wake(case, expected):
    wakes = []
    node = NS(paused=False, active_epoch=3, epoch=3, future=None, names=['joint1'],
        first_generated_epoch=2, published={123: dict(epoch=3, kind='t')},
        trajectory_rejections=0, timed=None, wake=NS(trigger=lambda: wakes.append('wake')))
    if case == 'later': node.first_generated_epoch = 3
    if case == 'paused': node.paused = True
    if case == 'old_epoch': node.active_epoch = 1
    if case == 'pending_service': node.future = object()
    if case == 'old_source': node.published[123]['epoch'] = 1
    if case == 'unknown_source': node.published.clear()
    trajectory = NS(sources=[NS(kind='t', input_ns=123)])
    msg = NS(joint_names=['joint1'], header=NS(stamp=NS(sec=1, nanosec=0), frame_id='trace'),
             points=[NS(time_from_start=NS(sec=0, nanosec=n), positions=[], velocities=[]) for n in (10, 20)])
    callback = method('trajectory', dict(TimedTrajectory=lambda *args: trajectory, clock=lambda: 25))
    callback(node, msg)
    assert len(wakes) == expected
    if case in ('old_source', 'unknown_source'):
        assert node.trajectory_rejections == 1 and node.timed is None
