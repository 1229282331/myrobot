from pathlib import Path
import io
import sys
from types import SimpleNamespace

import pytest

from e6_phosphobot.cpu_policy import SessionCpuPolicy, configured_governor


def policy(root, name='policy0', original='schedutil'):
    path = root/name
    path.mkdir()
    (path/'scaling_available_governors').write_text('schedutil performance powersave')
    (path/'scaling_governor').write_text(original)
    (path/'scaling_min_freq').write_text('300000')
    (path/'scaling_max_freq').write_text('1500000')
    return path/'scaling_governor'


def test_scoped_policy_restores_multiple_originals_without_changing_limits(tmp_path):
    paths = [policy(tmp_path), policy(tmp_path, 'policy4', 'powersave')]
    events = []
    owned = SessionCpuPolicy('performance', root=tmp_path,
                             event=lambda name, **data: events.append((name, data)))
    owned.apply()
    assert [p.read_text().strip() for p in paths] == ['performance']*2
    owned.restore()
    owned.restore()  # cleanup is idempotent
    assert [p.read_text().strip() for p in paths] == ['schedutil', 'powersave']
    assert all(p.parent.joinpath('scaling_min_freq').read_text() == '300000' for p in paths)
    assert all(p.parent.joinpath('scaling_max_freq').read_text() == '1500000' for p in paths)
    assert events[0][0] == 'cpu_policy_original'
    assert all(not p['changed'] for p in events[0][1]['policies'])
    assert all(p['restored'] for p in owned.snapshot()['policies'])


def test_already_performance_is_not_owned_or_restored(tmp_path):
    path = policy(tmp_path, original='performance')
    owned = SessionCpuPolicy('performance', root=tmp_path)
    owned.apply()
    path.write_text('powersave')
    owned.restore()
    assert path.read_text() == 'powersave'
    assert not owned.snapshot()['policies'][0]['changed']


def test_external_change_is_reported_and_not_overwritten(tmp_path):
    path = policy(tmp_path)
    owned = SessionCpuPolicy('performance', root=tmp_path)
    owned.apply()
    path.write_text('powersave')
    with pytest.raises(RuntimeError, match='externally changed'):
        owned.restore()
    assert path.read_text() == 'powersave'


def test_failure_on_later_policy_still_restores_changed_policy(tmp_path, monkeypatch):
    first = policy(tmp_path)
    second = policy(tmp_path, 'policy4')
    write = Path.write_text
    def fail_second(path, text, **kwargs):
        if path == second and text.strip() == 'performance':
            raise OSError('injected write failure')
        return write(path, text, **kwargs)
    monkeypatch.setattr(Path, 'write_text', fail_second)
    owned = SessionCpuPolicy('performance', root=tmp_path)
    with pytest.raises(OSError, match='injected'):
        owned.apply()
    assert first.read_text().strip() == 'performance'
    owned.restore()
    assert first.read_text().strip() == second.read_text().strip() == 'schedutil'


def test_preflight_missing_or_unsupported_never_changes_other_policies(tmp_path):
    with pytest.raises(RuntimeError, match='unavailable'):
        SessionCpuPolicy('performance', root=tmp_path).apply()
    first = policy(tmp_path)
    second = policy(tmp_path, 'policy4')
    second.parent.joinpath('scaling_available_governors').write_text('schedutil')
    with pytest.raises(RuntimeError, match='unavailable'):
        SessionCpuPolicy('performance', root=tmp_path).apply()
    assert first.read_text() == second.read_text() == 'schedutil'


def test_default_does_not_touch_sysfs_and_bad_config_rejected():
    assert configured_governor({}) == 'unchanged'
    owned = SessionCpuPolicy('unchanged', root='/nonexistent')
    assert owned.apply()['policies'] == owned.restore()['policies'] == []
    for value in (None, True, {}, [], 'overclock'):
        with pytest.raises(ValueError):
            configured_governor({'cpu_governor': value})


@pytest.mark.parametrize('failure', ['gateway_exit', 'gateway_launch', 'gateway_stuck'])
def test_supervisor_owns_policy_through_child_cleanup(tmp_path, monkeypatch, failure):
    from e6_phosphobot import cpu_policy, session_runner
    cpus = tmp_path/'cpus'; cpus.mkdir()
    governor = policy(cpus)
    events = []
    monkeypatch.setattr(cpu_policy, 'SessionCpuPolicy',
        lambda mode, event: SessionCpuPolicy(mode, root=cpus, event=event))
    monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=lambda *a: None))
    monkeypatch.setattr(session_runner, 'open', lambda *a: io.StringIO(), raising=False)
    monkeypatch.setattr(session_runner, 'adopt_descendants', lambda: None)
    monkeypatch.setattr(session_runner, 'close_adopted', lambda: events.append('descendants_closed'))
    monkeypatch.setattr(session_runner.signal, 'signal', lambda *a: None)
    monkeypatch.setattr(session_runner.threading, 'Thread', lambda **kw: SimpleNamespace(start=lambda: None))
    read = Path.read_text
    monkeypatch.setattr(Path, 'read_text', lambda p, **kw:
        'test-boot' if p.as_posix() == '/proc/sys/kernel/random/boot_id' else read(p, **kw))
    monkeypatch.setattr(session_runner.subprocess, 'check_output', lambda *a, **kw: '')
    monkeypatch.setattr(session_runner.subprocess, 'run', lambda *a, **kw: SimpleNamespace(returncode=1))
    def launch(args, **kwargs):
        name = args[3].rsplit('.', 1)[1]
        assert governor.read_text().strip() == 'performance'
        if name == 'gateway' and failure == 'gateway_launch':
            raise RuntimeError('injected launch failure')
        return SimpleNamespace(pid=len(events)+100, name=name,
            poll=lambda: 1 if name == 'gateway' and failure != 'gateway_stuck' else None)
    monkeypatch.setattr(session_runner.subprocess, 'Popen', launch)
    def close(child):
        assert governor.read_text().strip() == 'performance'
        events.append(child.name+'_closed')
        if child.name == 'gateway' and failure == 'gateway_stuck':
            raise RuntimeError('resources retained')
        child.poll = lambda: 0
        return 0
    monkeypatch.setattr(session_runner, 'close_child', close)
    monkeypatch.setattr(session_runner, 'check_children', lambda *a: (_ for _ in ()).throw(RuntimeError('injected exit')))
    config = dict(session='test', stage='shadow', runtime=str(tmp_path/'run'),
        release=str(tmp_path/'release'), source_manifest={}, rootfs=str(tmp_path/'ros'),
        motion_engine='native_servop', cpu_governor='performance', gateway_dependencies='/unused',
        camera_sdk='/unused', camera_config='/unused', camera_tools='/unused')
    result = session_runner.run(config)
    assert result.get('error')
    if failure == 'gateway_stuck':
        assert not result['clean'] and governor.read_text().strip() == 'performance'
        assert 'camera_service_closed' not in events
        assert any('governor retained' in error for error in result['cleanup_errors'])
    else:
        assert result['clean'] and governor.read_text().strip() == 'schedutil'
        assert events[-1] == 'descendants_closed'
        assert result['cpu_policy']['policies'][0]['restored']
