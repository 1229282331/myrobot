"""Deterministic no-device reproduction using archived v182 input payloads.

The candidate and device are test doubles. This compares executable baseline
and current gateway code, not physical timing or a new hardware qualification.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import threading
import types
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'teleop-platform/src'),str(ROOT/'teleop-platform/tests'),
              str(ROOT/'teleop-platform/upstream/dobot-sdk')]
import test_gateway
from test_moveit_bridge import Bridge,row
from e6_phosphobot.gateway import NativeExecutor


def run_case(executor_type,group,outside):
    with patch.object(test_gateway,'NativeExecutor',executor_type):
        executor,device,send,step,clock=test_gateway.setup(motion_profile='normal',moveit_bundle='unused-test-bundle')
    executor.moveit=Bridge()
    first=next(e for e in group['trace'] if e['kind']=='input' and e['seq']==group['first_sequence'])
    released=next(e for e in group['trace'] if e['kind']=='input' and e['seq']==group['release_sequence'])
    send(**first['intent'])
    executor.moveit.row=row(clock[0],executor.session.sequence);step()
    message=dict(op='intent',seq=executor.session.sequence+1,echo_ns=executor.session.clock_tick(),
                 source='keyboard',client_sample_ms=0,intent=deepcopy(released['intent']))
    candidate=row(clock[0],executor.session.sequence)
    if outside:candidate['position_m']=[1,0,.4]
    entered=threading.Event();resume=threading.Event();failures=[]
    def take():
        entered.set()
        if not resume.wait(3):raise TimeoutError('Reproduction did not release computation')
        return candidate
    executor.moveit.take=take
    def tick():
        try:step()
        except BaseException as exc:failures.append(repr(exc))
    worker=threading.Thread(target=tick);worker.start()
    try:
        if not entered.wait(3):raise TimeoutError('Computation barrier not reached')
        before=(executor.session.sequence,executor.session.received_ns)
        if hasattr(executor,'notify_received'):executor.notify_received('page',message)
        assert before==(executor.session.sequence,executor.session.received_ns)
    finally:resume.set();worker.join(3)
    assert not worker.is_alive() and not failures,failures
    before_handler=[c[0] for c in device.calls]
    blocked=sum(e['event']=='workspace_blocked' for e in executor.events)
    executor.receive(message);step()
    return dict(commands_before_serialized_release=before_handler,
        boundary_events_before_serialized_release=blocked,
        final_commands=[c[0] for c in device.calls],ready_after_release=executor.session.ready)


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--output',type=Path,required=True)
    args=cli.parse_args()
    relative='teleop-platform/src/e6_phosphobot/gateway.py'
    baseline_ref='f9052da'
    baseline_source=subprocess.check_output(['git','show',f'{baseline_ref}:{relative}'],cwd=ROOT)
    module=types.ModuleType('e6_phosphobot.gateway_cancellation_baseline')
    module.__file__=str(ROOT/relative)
    sys.modules[module.__name__]=module
    exec(compile(baseline_source,module.__file__,'exec'),module.__dict__)
    fixture=ROOT/'docs/teleop-rebuild/evidence/live-jazzy-v182/input-detail.json'
    groups=json.loads(fixture.read_text(encoding='utf8'))
    cases=[]
    for group in groups:
        for outside in (False,True):
            before=run_case(module.NativeExecutor,group,outside)
            after=run_case(NativeExecutor,group,outside)
            assert after['commands_before_serialized_release']==['ServoJ','Stop']
            assert after['final_commands']==['ServoJ','Stop'] and after['ready_after_release']
            assert after['boundary_events_before_serialized_release']==0
            if outside:assert before['boundary_events_before_serialized_release']==1
            else:assert before['commands_before_serialized_release']==['ServoJ','ServoJ']
            cases.append(dict(archived_release_sequence=group['release_sequence'],
                              candidate_outside_workspace=outside,before=before,after=after))
    result=dict(kind='offline_thread_interleaving_reproduction',baseline_ref=baseline_ref,
        baseline_gateway_sha256=hashlib.sha256(baseline_source).hexdigest(),
        current_gateway_sha256=hashlib.sha256((ROOT/relative).read_bytes()).hexdigest(),
        input_fixture=str(fixture.relative_to(ROOT)),input_fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),
        cases=cases,passed=True,device_io=False,
        scope='Archived intent payloads; synthetic candidate and device, forced arrival during computation. No hardware timing claim.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(passed=True,cases=len(cases),device_io=False)))


if __name__=='__main__':main()
