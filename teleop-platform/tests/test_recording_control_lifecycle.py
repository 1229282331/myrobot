"""Recording lifecycle through the real journal; device writes stay in a fixture."""
import hashlib

import pytest

from test_episodes import setup as episode_setup, wait
from test_gateway import setup


@pytest.mark.parametrize('outcome', ['success', 'failure', 'aborted'])
def test_sealed_transfer_does_not_lock_control_or_rewrite_recording(episode_setup, outcome):
    episodes, _, _ = episode_setup
    executor, device, send, step, _ = setup('shadow')
    executor.episodes = episodes
    executor.receive(dict(op='record_start', task='offline recording regression',
                          seq=executor.session.sequence+1, echo_ns=executor.session.clock_tick(),
                          source='keyboard', client_sample_ms=0))
    wait(lambda: episodes.state()['phase'] == 'capturing')
    episodes.put(dict(event='cycle', at_ns=1, state={'stationary': True}))
    executor.receive(dict(op='record_finish', result=outcome,
                          seq=executor.session.sequence+1, echo_ns=executor.session.clock_tick(),
                          source='keyboard', client_sample_ms=0))
    assert not executor.session.ready
    send(); send('prepare')
    assert not executor.session.ready  # Sealing still owns the settled window.
    wait(lambda: episodes.state()['phase'] == 'transfer_pending')
    directory = episodes.directory
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in directory.rglob('*') if p.is_file() and p.name != 'transfer.json'}
    send(); send('prepare')
    assert executor.session.ready  # No transfer acknowledgement required.
    for _ in range(20):
        send(deadman=True, linear_m_s=[.002, 0, 0]); step()
        executor.check_recording()
        assert executor.session.ready
    assert executor.candidate is not None and not device.calls
    with pytest.raises(ValueError):
        episodes.request('start', task='overlapping recording remains disabled')
    with pytest.raises(ValueError, match='校验'):
        episodes.transferred(directory.name, 'wrong checksum')
    executor.check_recording()
    assert executor.session.ready and episodes.state()['phase'] == 'transfer_pending'
    send('stop')
    assert not executor.session.ready
    assert episodes.state()['result'] == outcome
    assert episodes.state()['phase'] == 'transfer_pending'
    episodes.transferred(directory.name, episodes.state()['bundle_sha256'])
    assert episodes.state()['result'] == outcome
    assert not executor.session.ready  # Receipt never starts control.
    assert frozen == {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen}


def test_recording_maintenance_still_aborts_bad_camera_and_stops_sealing(episode_setup):
    episodes, _, _ = episode_setup
    executor, _, send, _, _ = setup('shadow')
    executor.episodes = episodes
    episodes.request('start', task='bad camera regression', requested_ns=1)
    wait(lambda: episodes.state()['phase'] == 'capturing')
    executor.camera = lambda: dict(valid=False)
    executor.check_recording()
    assert episodes.state()['phase'] == 'sealing'
    executor.check_recording()
    assert not executor.session.ready
    wait(lambda: episodes.state()['phase'] == 'transfer_pending')
    assert episodes.state()['result'] == 'aborted'
    assert 'RGB' in episodes.state()['error']
