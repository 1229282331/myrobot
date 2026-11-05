import io
import json
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace

import pytest

from e6_phosphobot.launcher import gateway_ready, listener_owner, LocalServices, make_payload
from e6_phosphobot.session_runner import verified_files, close_child


def test_cold_start_wait_is_configurable_and_does_not_extend_feedback_validity():
    from e6_phosphobot.launcher import startup_timeout_seconds
    assert startup_timeout_seconds({})==120
    assert startup_timeout_seconds({'startup_timeout_s':180})==180
    config=dict(startup_timeout_s=180,motion_engine='native_servop')
    state=dict(stage='shadow',feedback={'connected':True,'age_ms':99},
        camera={'valid':True},capabilities=dict(motion_engine='native_servop',
        servo_gain=500,servo_aheadtime=50,servo_duration_s=.1))
    assert gateway_ready(state,'shadow',servo_config=config)
    state['feedback']['age_ms']=100
    assert not gateway_ready(state,'shadow',servo_config=config)
    for value in (None,True,'120',float('nan'),float('inf'),0,9,301):
        with pytest.raises(ValueError):startup_timeout_seconds({'startup_timeout_s':value})


def test_startup_fixed_motion_engine_is_explicit_and_low_speed_native_only():
    from e6_phosphobot.session_runner import motion_engine_option
    assert motion_engine_option({'rootfs': '/ros'}) == ['--moveit-bundle', str(Path('/ros/qualification'))]
    assert motion_engine_option({'motion_engine': 'native_servop'}) == []
    for engine in ('native', None, [], True):
        with pytest.raises(ValueError):
            motion_engine_option({'motion_engine': engine})
    for profile in ('intermediate', 'normal'):
        with pytest.raises(ValueError, match='commissioning'):
            motion_engine_option({'motion_engine': 'native_servop', 'motion_profile': profile})


def test_native_readiness_checks_actual_route_without_ros_model():
    config = {'motion_engine': 'native_servop'}
    state = dict(stage='shadow', feedback={'connected': True, 'age_ms': 8},
                 camera={'valid': True}, model=None, capabilities=dict(
                     motion_engine='native_servop', servo_gain=500, servo_aheadtime=50, servo_duration_s=.1))
    assert gateway_ready(state, 'shadow', servo_config=config)
    assert not gateway_ready(state, 'live', servo_config=config)
    assert not gateway_ready(state, 'shadow', servo_config={})
    state['capabilities']['motion_engine'] = 'moveit_servo'
    assert not gateway_ready(state, 'shadow', servo_config=config)
    assert not gateway_ready(state, 'shadow', servo_config={})
    state['model'] = {'ready': True}
    assert gateway_ready(state, 'shadow', servo_config={})
    state['capabilities']['motion_engine'] = 'native_servop'
    for field in ('camera', 'feedback'):
        assert not gateway_ready(dict(state, **{field: None}), 'shadow', servo_config=config)


def test_launcher_preserves_explicit_official_servo_gain():
    from e6_phosphobot.session_runner import servo_gain_option
    assert servo_gain_option({}) == ['--servo-gain', '500']
    assert servo_gain_option({'servo_gain': 750}) == ['--servo-gain', '750']
    for gain in (True, '750', None, float('nan'), 1001):
        with pytest.raises(ValueError):
            servo_gain_option({'servo_gain': gain})


def test_launcher_preserves_explicit_official_servo_aheadtime_and_checks_readback():
    from e6_phosphobot.session_runner import servo_aheadtime_option
    assert servo_aheadtime_option({}) == ['--servo-aheadtime', '50']
    assert servo_aheadtime_option({'servo_aheadtime': 20}) == ['--servo-aheadtime', '20']
    for value in (True, '20', None, float('nan'), 0, 19, 100):
        with pytest.raises(ValueError):servo_aheadtime_option({'servo_aheadtime': value})
    state=dict(stage='shadow',feedback={'connected':True,'age_ms':8},camera={'valid':True},
        model={'ready':True},capabilities={'motion_engine':'moveit_servo','servo_gain':500,'servo_aheadtime':20,'servo_duration_s':.1})
    assert gateway_ready(state,'shadow',servo_config={'servo_aheadtime':20})
    assert not gateway_ready(state,'shadow',servo_config={})
    assert not gateway_ready(state,'shadow',servo_config={'servo_aheadtime':20,'servo_gain':750})


def test_launcher_duration_must_match_remote_readback():
    from e6_phosphobot.session_runner import servo_duration_option
    assert servo_duration_option({})==['--servo-duration-s','0.1']
    assert servo_duration_option({'servo_duration_s':.1})==['--servo-duration-s','0.1']
    for value in (None,True,'0.1',float('nan'),float('inf'),0,.03,1):
        with pytest.raises(ValueError):servo_duration_option({'servo_duration_s':value})
    state=dict(stage='shadow',feedback={'connected':True,'age_ms':8},camera={'valid':True},
        model={'ready':True},capabilities={'motion_engine':'moveit_servo','servo_gain':500,'servo_aheadtime':50,'servo_duration_s':.1})
    assert gateway_ready(state,'shadow',servo_config={'servo_duration_s':.1})
    assert gateway_ready(state,'shadow',servo_config={})
    assert not gateway_ready(state,'shadow',servo_config={'servo_duration_s':.05})
    del state['capabilities']['servo_duration_s']
    assert not gateway_ready(state,'shadow',servo_config={'servo_duration_s':.1})
    assert not gateway_ready(state,'shadow',servo_config={})


def test_readiness_is_not_tcp_listen_or_device_enable():
    value = dict(stage='shadow', feedback={'connected': True, 'age_ms': 8},
                 camera={'valid': True}, model={'ready': True})
    assert gateway_ready(value, 'shadow')
    assert not gateway_ready(value, 'live')
    for field in ('feedback', 'camera', 'model'):
        assert not gateway_ready(dict(value, **{field: None}), 'shadow')
    for age in (None, -1, 101, float('nan')):
        assert not gateway_ready(dict(value, feedback={'connected': True, 'age_ms': age}), 'shadow')


def test_preview_measurement_is_opt_in_and_requires_remote_marker():
    from e6_phosphobot.session_runner import preview_measurement_option
    assert preview_measurement_option({})==[]
    assert preview_measurement_option({'latency_marker':True})==['--latency-marker']
    for value in (None,1,'true'):
        with pytest.raises(ValueError):preview_measurement_option({'latency_marker':value})
    state=dict(stage='shadow',feedback={'connected':True,'age_ms':8},camera={'valid':True},
        model={'ready':True},capabilities={'motion_engine':'moveit_servo','servo_gain':500,'servo_aheadtime':50,'servo_duration_s':.1})
    assert not gateway_ready(state,'shadow',servo_config={'latency_marker':True})
    state['camera']['latency_marker']=True
    assert gateway_ready(state,'shadow',servo_config={'latency_marker':True})
    assert not gateway_ready(state,'shadow',servo_config={})


@pytest.mark.parametrize('overrides,expected', [
    ([], dict(servo_duration_s=.05, preview_codec='h264', latency_marker=False)),
    (['--servo-duration-s', '0.1', '--preview-codec', 'vp8'],
     dict(servo_duration_s=.1, preview_codec='vp8', latency_marker=False)),
    (['--latency-marker'], dict(servo_duration_s=.05, preview_codec='h264', latency_marker=True)),
])
def test_cli_passes_explicit_following_configuration(monkeypatch, tmp_path, overrides, expected):
    from e6_phosphobot import launcher
    path=tmp_path/'config.json'
    path.write_text(json.dumps(dict(servo_duration_s=.05, preview_codec='h264', latency_marker=False)))
    calls=[]
    monkeypatch.setattr(sys, 'argv', ['launcher','--config',str(path),*overrides])
    monkeypatch.setattr(launcher, 'ssh_factory', lambda *a: None)
    monkeypatch.setattr(launcher, 'run', lambda config, **kw: calls.append(config) or {'clean':True})
    with pytest.raises(SystemExit) as exit:
        launcher.main()
    assert exit.value.code==0
    assert {k:calls[0][k] for k in expected}==expected
    assert calls[0]['stage']=='shadow' and calls[0]['motion_engine']=='moveit_servo'


def test_public_cli_materializes_followed_duration_and_cpp_selection(monkeypatch,tmp_path):
    from e6_phosphobot import launcher
    path=tmp_path/'config.json';path.write_text('{}')
    calls=[]
    monkeypatch.setattr(sys,'argv',['launcher','--config',str(path),'--moveit-api','cpp'])
    monkeypatch.setattr(launcher,'ssh_factory',lambda *a:None)
    monkeypatch.setattr(launcher,'run',lambda config,**kw:calls.append(config) or {'clean':True})
    with pytest.raises(SystemExit) as exit:launcher.main()
    assert exit.value.code==0
    assert calls[0]['stage']=='shadow' and calls[0]['servo_duration_s']==.1
    assert calls[0]['moveit_api']=='cpp' and not calls[0]['acceptance_inputs']


@pytest.mark.parametrize('workspace,override,expected', [
    ({'position_m': .05}, [], ('local', 'ros')),
    ({'position_m': .05, 'mode': 'model'}, [], ('model', 'cpp')),
    ({'position_m': .05, 'mode': 'model'}, ['--workspace-mode', 'local', '--moveit-api', 'cpp'], ('local', 'cpp')),
])
def test_public_cli_selects_workspace_solver_without_starting_devices(monkeypatch, tmp_path, workspace, override, expected):
    from e6_phosphobot import launcher
    path = tmp_path/'config.json'
    path.write_text(json.dumps({'test_workspace': workspace}))
    calls = []
    monkeypatch.setattr(sys, 'argv', ['launcher', '--config', str(path), *override])
    monkeypatch.setattr(launcher, 'ssh_factory', lambda *a: None)
    monkeypatch.setattr(launcher, 'run', lambda config, **kw: calls.append(config) or {'clean': True})
    with pytest.raises(SystemExit) as exit:
        launcher.main()
    assert exit.value.code == 0
    config = calls[0]
    assert (config['test_workspace'].get('mode', 'local'), config['moveit_api']) == expected
    assert config['test_workspace']['position_m'] == .05
    assert config['stage'] == 'shadow' and not config['acceptance_inputs']


@pytest.mark.parametrize('entry',['cli','launcher','gateway'])
def test_obsolete_live_duration_is_rejected_before_resources_or_password(entry,monkeypatch,tmp_path):
    import asyncio
    from e6_phosphobot import gateway,launcher
    from e6_phosphobot.session_runner import servo_duration_option
    # The old profile remains reproducible without commands in shadow mode.
    assert servo_duration_option({'stage':'shadow','servo_duration_s':.05})==['--servo-duration-s','0.05']
    def forbidden(*a,**kw):raise AssertionError('Must not access resources or prompt for credentials')
    monkeypatch.setattr(launcher,'ssh_factory',forbidden)
    monkeypatch.setattr(launcher,'LocalServices',forbidden)
    monkeypatch.setattr(gateway,'NativeExecutor',forbidden)
    config=dict(stage='live',servo_duration_s=.05,rootfs='/unused')
    with pytest.raises(ValueError,match='Live control requires Servo t=0.1'):
        if entry=='launcher':launcher.run(config,connect=forbidden)
        elif entry=='gateway':asyncio.run(gateway.run(SimpleNamespace(**config)))
        else:
            path=tmp_path/'config.json';path.write_text(json.dumps(config))
            monkeypatch.setattr(sys,'argv',['launcher','--config',str(path),'--stage','live','--password'])
            launcher.main()


def test_preview_codec_is_startup_fixed_and_remote_readback_must_match():
    from e6_phosphobot.session_runner import preview_codec_option
    assert preview_codec_option({})==['--preview-codec','h264']
    assert preview_codec_option({'preview_codec':'vp8'})==['--preview-codec','vp8']
    for value in (None,True,[],{},'hevc'):
        with pytest.raises(ValueError):preview_codec_option({'preview_codec':value})
    state=dict(stage='shadow',feedback={'connected':True,'age_ms':8},camera={'valid':True},
        model={'ready':True},capabilities={'motion_engine':'moveit_servo','servo_gain':500,'servo_aheadtime':50,'servo_duration_s':.1})
    assert gateway_ready(state,'shadow',servo_config={})
    assert not gateway_ready(state,'shadow',servo_config={'preview_codec':'vp8'})
    state['camera']['preview_codec']='vp8'
    assert gateway_ready(state,'shadow',servo_config={'preview_codec':'vp8'})
    assert not gateway_ready(state,'shadow',servo_config={})


def test_release_manifest_rejects_mutation_and_escape(tmp_path):
    import hashlib
    artifact = tmp_path/'source.py'; artifact.write_bytes(b'tested')
    manifest = {'source.py': hashlib.sha256(b'tested').hexdigest()}
    verified_files(tmp_path, manifest)
    artifact.write_bytes(b'changed')
    with pytest.raises(ValueError, match='differs'):
        verified_files(tmp_path, manifest)
    with pytest.raises(ValueError, match='outside'):
        verified_files(tmp_path, {'../other': '0'*64})


def test_startup_rejects_unexpected_speed_profile():
    value = dict(stage='shadow', feedback={'connected': True, 'age_ms': 8}, camera={'valid': True},
                 model={'ready': True}, capabilities={'motion_profile': {'name': 'commissioning'}})
    assert gateway_ready(value, 'shadow', 'commissioning')
    assert not gateway_ready(value, 'shadow', 'normal')


def test_cleanup_does_not_kill_a_stuck_device_owner():
    calls = []
    def wait(timeout):
        raise subprocess.TimeoutExpired('gateway', timeout)
    child = SimpleNamespace(pid=123, poll=lambda: None, send_signal=calls.append, wait=wait)
    with pytest.raises(RuntimeError, match='resources retained'):
        close_child(child, .01)
    assert calls == [signal.SIGTERM]


def test_already_exited_child_is_not_signalled():
    child = SimpleNamespace(poll=lambda: 0, wait=lambda timeout: 0)
    assert close_child(child) == 0


def test_reuse_requires_real_process_identity(monkeypatch, tmp_path):
    app = tmp_path/'scripts/app.py'; app.parent.mkdir(); app.touch()
    process = SimpleNamespace(cmdline=lambda: ['python', 'scripts/app.py', '--backend', 'hardware-control'],
                              cwd=lambda: str(tmp_path), pid=123, create_time=lambda: 42)
    psutil = SimpleNamespace(CONN_LISTEN='LISTEN', Process=lambda pid: process,
        net_connections=lambda kind: [SimpleNamespace(status='LISTEN', laddr=SimpleNamespace(port=8765), pid=123)])
    monkeypatch.setitem(sys.modules, 'psutil', psutil)
    assert listener_owner(8765, [app, '--backend', 'hardware-control'])['pid'] == 123
    assert listener_owner(8765, [(tmp_path/'legacy.py', app)])['pid'] == 123
    with pytest.raises(RuntimeError, match='another process'):
        listener_owner(8765, [tmp_path/'other.py'])
    assert listener_owner(8554, [app]) is None


def test_reused_local_service_is_never_terminated(monkeypatch, tmp_path):
    monkeypatch.setattr('e6_phosphobot.launcher.listener_owner', lambda *a: {'pid': 12})
    services = LocalServices(tmp_path, True)
    services.ensure(8765, ['unused'], ['app'])
    assert services.close() == [] and services.children == []
    assert services.reused == [{'port': 8765, 'pid': 12}]
    with pytest.raises(RuntimeError, match='already active'):
        LocalServices(tmp_path, False).ensure(8765, ['unused'], ['app'])


def test_owned_app_exits_via_stdin_and_media_via_own_process(tmp_path):
    events = []
    def child(name):
        return SimpleNamespace(pid=1, poll=lambda: None, stdin=SimpleNamespace(close=lambda: events.append(name+'-eof')),
            terminate=lambda: events.append(name+'-terminate'), wait=lambda timeout: events.append(name+'-wait'))
    services = LocalServices(tmp_path, False)
    services.children = [(child('media'), False), (child('app'), True)]
    assert services.close() == []
    assert events == ['app-eof', 'app-wait', 'media-terminate', 'media-wait']


def test_payload_is_explicit_sources_without_runtime_or_credentials():
    files, manifest = make_payload()
    assert 'dobot_sdk/api/motion.py' in files
    assert 'e6_phosphobot/session_runner.py' in files
    assert files.keys() == manifest.keys()
    from e6_phosphobot.assembly_model import verified_assets
    assets={'e6_phosphobot/assets/workspace/'+name for name in [*verified_assets()['files'],'manifest.json']}
    assert assets.issubset(files)
    assert all(name in assets or (name.endswith('.py') and name.startswith(('dobot_sdk/', 'e6_phosphobot/'))) for name in files)
    assert not any(name.endswith('reachability.json') for name in files)


def test_managed_pipe_eof_uses_notification_without_buffered_stdin():
    import os
    import threading
    from e6_phosphobot.managed_app import wait_for_parent
    read, write = os.pipe(); notified = threading.Event()
    worker = threading.Thread(target=wait_for_parent, args=(read, notified.set))
    try:
        worker.start()
        assert not notified.wait(.02)
        os.close(write); write = None
        assert notified.wait(1)
        worker.join(1)
        assert not worker.is_alive()
    finally:
        if write is not None: os.close(write)
        os.close(read)


@pytest.mark.parametrize('backend,enabled', [('hardware-control', False), ('upstream-simulation', True)])
def test_full_app_camera_discovery_only_runs_in_simulation(monkeypatch, backend, enabled):
    from e6_phosphobot import pc
    calls = []
    monkeypatch.setitem(sys.modules, 'phosphobot.app', SimpleNamespace(app=object(), start_server=lambda **kw: calls.append(kw)))
    monkeypatch.setitem(sys.modules, 'phosphobot.types', SimpleNamespace(SimulationMode=SimpleNamespace(headless='headless')))
    monkeypatch.setattr('e6_phosphobot.pc_bridge.install_hardware', lambda *a, **kw: None)
    monkeypatch.setattr(pc, 'install_simulation', lambda *a: None)
    monkeypatch.setattr(sys, 'argv', ['pc', '--backend', backend])
    pc.main()
    assert calls[0]['cameras'] is enabled
    assert calls[0]['max_opencv_index'] == 0
    assert calls[0]['can'] is False and calls[0]['host'] == '127.0.0.1'
