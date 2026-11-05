"""Owned WSL child exit is never mistaken for confirmed remote resource release."""
import io
import subprocess
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from e6_teleop.wsl_tunnel import WslTunnel


def make():
    with patch('e6_teleop.wsl_tunnel.sys.platform','win32'):
        return WslTunnel('192.168.1.13',remote_root='/owned',pc_rto_min_ms=20)


def test_existing_runtime_requires_explicit_verified_peer_route():
    with patch('e6_teleop.wsl_tunnel.sys.platform','win32'):
        with pytest.raises(ValueError):WslTunnel('192.168.1.13')


def test_child_exit_without_remote_confirmation_stays_unknown():
    t=make();t.child=SimpleNamespace(stdin=io.StringIO(),wait=lambda **kw:0,returncode=0)
    t.reader=SimpleNamespace(join=lambda **kw:None)
    result=t.close()
    assert t.child.stdin.closed
    assert result['remote_exited'] is False


def test_child_timeout_does_not_kill_unrelated_wsl_or_claim_success():
    t=make()
    def wait(**kw):raise subprocess.TimeoutExpired('owned child',15)
    t.child=SimpleNamespace(stdin=io.StringIO(),wait=wait)
    result=t.close()
    assert not result['remote_exited'] and 'not confirmed' in result['reason']


def test_confirmed_child_report_is_preserved():
    t=make();t.child=SimpleNamespace(stdin=io.StringIO(),wait=lambda **kw:0,returncode=0)
    t.reader=SimpleNamespace(join=lambda **kw:None)
    report=dict(remote_owned=True,remote_exited=True,exit_code=0)
    t.messages.put(dict(event='closed',result=report))
    assert t.close()==report


def test_route_created_before_failed_child_launch_is_restored():
    t=make()
    with patch.object(t,'configure_route',side_effect=[dict(mode='create'),dict(mode='remove')]) as route,patch('e6_teleop.wsl_tunnel.linux_path',return_value='/mnt/c/project/ssh_runtime.py'),patch('e6_teleop.wsl_tunnel.subprocess.CREATE_NO_WINDOW',0,create=True),patch('e6_teleop.wsl_tunnel.subprocess.Popen',side_effect=OSError('launch failed')):
        with pytest.raises(OSError):t.start()
        assert [call.args[0] for call in route.call_args_list]==['create','remove']
        assert not t.route_owned


def test_route_cleanup_failure_stays_recorded_not_silently_successful():
    t=make();t.route_owned=True
    with patch.object(t,'configure_route',side_effect=RuntimeError('external route changed')):
        report=t.close()
    assert t.route_owned and report['pc_route_cleanup']['restored'] is False
