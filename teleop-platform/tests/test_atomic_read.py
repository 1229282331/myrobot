import errno
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from e6_phosphobot import camera_archive as module


@pytest.mark.parametrize('platform',['nt','posix'])
def test_transient_crt_access_denial_is_bounded_and_windows_only(tmp_path,monkeypatch,platform):
    now=[0.];calls=[]
    monkeypatch.setattr(module,'os',SimpleNamespace(name=platform))
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s)))
    def read(*args,**kwargs):
        calls.append(1)
        if len(calls)<3:raise PermissionError(errno.EACCES,'rename in progress')
        return '{"request_id":"new-value"}'
    monkeypatch.setattr(Path,'read_text',read)
    if platform=='nt':
        assert module.read_atomic_json(tmp_path/'state.json')=={'request_id':'new-value'}
        assert len(calls)==3 and now[0]==.004
    else:
        with pytest.raises(PermissionError):module.read_atomic_json(tmp_path/'state.json')
        assert len(calls)==1 and now[0]==0


def test_persistent_access_denial_fails_in_50ms_without_stale_fallback(tmp_path,monkeypatch):
    now=[0.]
    monkeypatch.setattr(module,'os',SimpleNamespace(name='nt'))
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s)))
    def read(*args,**kwargs):raise PermissionError(errno.EACCES,'ACL denied')
    monkeypatch.setattr(Path,'read_text',read)
    with pytest.raises(PermissionError,match='ACL denied'):module.read_atomic_json(tmp_path/'state.json')
    assert .05<=now[0]<.053


def test_invalid_json_is_not_retried_or_replaced_with_previous_value(tmp_path,monkeypatch):
    path=tmp_path/'state.json';path.write_text('broken')
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:0,sleep=lambda _:pytest.fail('Unexpected retry')))
    with pytest.raises(json.JSONDecodeError):module.read_atomic_json(path)
