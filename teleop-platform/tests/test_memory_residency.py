from types import SimpleNamespace
import sys
import pytest
from e6_phosphobot import memory_residency as module
from e6_phosphobot.session_runner import memory_residency_option
from e6_phosphobot.launcher import gateway_ready


def fixture(monkeypatch, *, size=100*1024**2, available=2*1024**3,
            already_locked=0, result=0, readback=100*1024**2, limit=400*1024**2):
    calls=[];reads=[0]
    def memory():
        reads[0]+=1
        return dict(VmSize=size,VmLck=already_locked if reads[0]==1 else readback),dict(MemAvailable=available)
    monkeypatch.setattr(module,'sys',SimpleNamespace(platform='linux'))
    monkeypatch.setitem(sys.modules,'resource',SimpleNamespace(RLIMIT_MEMLOCK=8,RLIM_INFINITY=-1,getrlimit=lambda _: (limit,limit)))
    monkeypatch.setattr(module,'_memory',memory)
    monkeypatch.setattr(module.ctypes,'CDLL',lambda *a,**k:SimpleNamespace(
        mlockall=lambda mode:(calls.append(('lock',mode)) or result),
        munlockall=lambda:(calls.append(('unlock',)) or 0)))
    return calls


def test_off_does_not_touch_os_or_another_owner(monkeypatch):
    monkeypatch.setattr(module,'_memory',lambda:pytest.fail('off must not inspect or change locks'))
    with module.resident_current() as state:
        assert state==dict(mode='off',locked_bytes=0)


@pytest.mark.parametrize('body_error',[False,True])
def test_current_lock_is_before_body_and_always_released(monkeypatch,body_error):
    calls=fixture(monkeypatch)
    try:
        with module.resident_current('current') as state:
            assert calls==[('lock',1)]
            assert state['locked_bytes']==100*1024**2 and not state['future_allocations_locked']
            if body_error:raise LookupError('body failed')
    except LookupError:
        assert body_error
    assert calls==[('lock',1),('unlock',)]


@pytest.mark.parametrize('kwargs',[
    {'already_locked':4096},{'size':351*1024**2},{'limit':99*1024**2},
    {'available':600*1024**2}])
def test_precondition_failure_cannot_enter_device_body_or_unlock_another_owner(monkeypatch,kwargs):
    calls=fixture(monkeypatch,**kwargs)
    with pytest.raises(RuntimeError):
        with module.resident_current('current'):pytest.fail('Must reject before devices')
    assert calls==[]


def test_failed_lock_cannot_open_devices(monkeypatch):
    calls=fixture(monkeypatch,result=-1)
    with pytest.raises(OSError):
        with module.resident_current('current'):pytest.fail('Must not silently continue')
    assert calls==[('lock',1),('unlock',)]


def test_failed_readback_releases_own_lock(monkeypatch):
    calls=fixture(monkeypatch,readback=0)
    with pytest.raises(RuntimeError,match='without locked pages'):
        with module.resident_current('current'):pytest.fail('Unverified residency')
    assert calls==[('lock',1),('unlock',)]


def test_platform_and_configuration_are_explicit(monkeypatch):
    monkeypatch.setattr(module,'sys',SimpleNamespace(platform='win32'))
    with pytest.raises(RuntimeError,match='Linux'):
        with module.resident_current('current'):pass
    assert memory_residency_option({})==['--memory-residency','off']
    assert memory_residency_option({'memory_residency':'current'})==['--memory-residency','current']
    with pytest.raises(ValueError):memory_residency_option({'memory_residency':True})


def test_ready_requires_actual_residency_readback_when_requested():
    config=dict(motion_engine='native_servop',memory_residency='current')
    value=dict(stage='shadow',feedback=dict(connected=True,age_ms=1),camera=dict(valid=True),
        capabilities=dict(motion_engine='native_servop',servo_gain=500,servo_aheadtime=50,servo_duration_s=.1))
    assert not gateway_ready(value,'shadow',servo_config=config)
    value['capabilities']['memory_residency']=dict(mode='current',locked_bytes=0)
    assert not gateway_ready(value,'shadow',servo_config=config)
    value['capabilities']['memory_residency']['locked_bytes']=True
    assert not gateway_ready(value,'shadow',servo_config=config)
    value['capabilities']['memory_residency']['locked_bytes']=4096
    assert gateway_ready(value,'shadow',servo_config=config)
    assert not gateway_ready(value,'shadow',servo_config=dict(config,memory_residency='off'))
