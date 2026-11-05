import gc
import pytest
import sys
from e6_phosphobot.runtime_timing import RuntimeTiming


@pytest.mark.parametrize('initial', [.005, .0005])
def test_gateway_thread_handoff_restores_existing_policy_after_failure(initial):
    from e6_phosphobot.runtime_timing import cooperative_thread_scheduling
    before = sys.getswitchinterval()
    try:
        sys.setswitchinterval(initial)
        previous = sys.getswitchinterval()
        with pytest.raises(RuntimeError, match='worker failed'):
            with cooperative_thread_scheduling() as report:
                assert sys.getswitchinterval() == pytest.approx(min(previous, .001), abs=1e-6)
                assert report['previous_s'] == previous
                assert report['interval_s'] == sys.getswitchinterval()
                raise RuntimeError('worker failed')
        assert sys.getswitchinterval() == previous
    finally:
        sys.setswitchinterval(before)


def test_retains_callback_gap_and_work_separately_without_device_policy():
    wall=[0];cpu=[0];p=RuntimeTiming(lambda:wall[0],lambda:cpu[0])
    p.begin();wall[0]=2_000_000;cpu[0]=1_000_000;p.end()
    wall[0]=224_000_000;cpu[0]=3_000_000;p.begin()
    wall[0]=250_000_000;cpu[0]=23_000_000;p.end()
    s=p.snapshot()
    assert s['max_callback_gap_ms']==222 and s['max_callback_work_ms']==26
    assert [x['kind'] for x in s['slow']]==['between_callbacks','callback_work']
    assert [x['process_cpu_ms'] for x in s['slow']]==[2,20]


def test_slow_segments_distinguish_publish_wait_from_cpu_work():
    wall=[0];cpu=[0];p=RuntimeTiming(lambda:wall[0],lambda:cpu[0]);p.begin()
    wall[0]=1_000_000;cpu[0]=1_000_000;p.mark('prepare_feedback')
    wall[0]=231_000_000;cpu[0]=2_000_000;p.mark('publish_feedback')
    wall[0]=232_000_000;cpu[0]=3_000_000;p.end()
    segments=p.snapshot()['slow'][0]['segments']
    assert segments==[dict(stage='publish_feedback',start_ns=1_000_000,end_ns=231_000_000,
                          wall_ms=230.,process_cpu_ms=1.)]


def test_gc_intervals_are_bounded_and_original_timestamps_are_preserved():
    now=[0];p=RuntimeTiming(lambda:now[0],lambda:0)
    p.on_gc('stop',{'generation':2})
    for n in range(12):
        p.on_gc('start',{'generation':2});now[0]+=50_000_000
        p.on_gc('stop',{'generation':2,'collected':3,'uncollectable':0})
    s=p.snapshot();assert s['gc_count']==[0,0,12] and len(s['gc_slow'])==8
    assert s['gc_slow'][0]['start_ns']==200_000_000
    assert s['gc_slow'][-1]['end_ns']==600_000_000
    s['gc_count'][2]=0
    assert p.snapshot()['gc_count'][2]==12


def test_installs_and_removes_only_owned_callback():
    before=list(gc.callbacks);p=RuntimeTiming()
    try:
        p.install();p.install();assert gc.callbacks==before+[p.on_gc]
        p.on_gc('start',{'generation':0});p.on_gc('stop',{'generation':0})
    finally:p.close();p.close()
    assert gc.callbacks==before


def test_gateway_diagnostics_do_not_fill_history_with_normal_20hz_waits():
    wall=[0];p=RuntimeTiming(lambda:wall[0],lambda:0,gap_slow_ms=100)
    for _ in range(12):
        p.begin();wall[0]+=1_000_000;p.end();wall[0]+=49_000_000
    assert p.snapshot()['slow']==[]
    wall[0]+=150_000_000;p.begin();wall[0]+=25_000_000;p.end()
    s=p.snapshot()
    assert [(r['kind'],r['wall_ms']) for r in s['slow']]==[
        ('between_callbacks',199),('callback_work',25)]
    assert s['max_callback_gap_ms']==199


def test_frozen_imports_still_collect_runtime_cycles_and_restore_after_failure():
    # A subprocess prevents the test runner's own imported graph from becoming
    # part of the worker's lifetime policy.
    import os, subprocess, sys
    from pathlib import Path
    source=str(Path(__file__).resolve().parents[1]/'src')
    code='''
import gc, weakref
from e6_phosphobot.runtime_timing import frozen_startup_heap
class Cycle: pass
before=(gc.isenabled(),gc.get_threshold())
try:
 with frozen_startup_heap() as report:
  assert report['frozen_objects']>0 and gc.get_freeze_count()>0
  obj=Cycle();obj.self=obj;ref=weakref.ref(obj);del obj
  gc.collect(2)
  assert ref() is None
  assert (gc.isenabled(),gc.get_threshold())==before
  raise ValueError('simulate worker exit')
except ValueError: pass
assert gc.get_freeze_count()==0
assert (gc.isenabled(),gc.get_threshold())==before
'''
    subprocess.run([sys.executable,'-c',code],env=dict(os.environ,PYTHONPATH=source),check=True)
