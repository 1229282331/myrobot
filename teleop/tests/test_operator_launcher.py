"""An operator entry point cannot widen accepted capabilities or skip gates."""
from pathlib import Path
import pytest
from e6_teleop import operator_launcher as launch


def profile():
    return dict(media_hz=5,feedback_framing='length-envelope',recovery_policy='bounded',
        wifi_low_latency=True,wifi_reorder_ms=10,ssh_rto_min_ms=20,
        pc_transport=dict(platform='win32-wsl-ssh'))


def test_startup_uses_only_accepted_configuration(monkeypatch,tmp_path):
    monkeypatch.setattr(launch,'verify_profile',lambda path:profile())
    args=launch.arguments(tmp_path/'acceptance.json')
    assert args[args.index('--controller-role')+1]=='operator'
    assert args[args.index('--control-stage')+1]=='live'
    assert '--harness-limits' not in args and '--pc-rto-min-ms' in args
    assert args[args.index('--recovery-policy')+1]=='bounded'


def test_incomplete_acceptance_cannot_build_a_startup_command(monkeypatch,tmp_path):
    def reject(_):raise ValueError('Hardware acceptance is incomplete')
    monkeypatch.setattr(launch,'verify_profile',reject)
    with pytest.raises(ValueError,match='incomplete'):launch.arguments(tmp_path/'profile.json')


def test_local_bundle_hash_is_checked_before_connecting(monkeypatch,tmp_path):
    p=profile();p.update(kinematics_kind='controller-local',kinematics_profile_file='model.json',kinematics_profile_sha256='0'*64)
    (tmp_path/'model.json').write_text('{}')
    monkeypatch.setattr(launch,'verify_profile',lambda _:p)
    with pytest.raises(ValueError,match='bundle changed'):launch.arguments(tmp_path/'profile.json')
