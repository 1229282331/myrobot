"""Cross-session recovery through the real media worker, with no device owner."""
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
import zipfile

import pytest

from e6_phosphobot.camera_archive import atomic_json
from e6_phosphobot.episode_http import start_server
from e6_phosphobot.episode_transfer import TransferWorker, digest_file
from e6_phosphobot.retained_episodes import RetainedEpisodes


EPISODE='20261009T120739Z_fixture0'
SESSION='20261009-195722-11d544a7'


def saved_session(root, name=SESSION, episode=EPISODE, result='aborted'):
    session=root/name;directory=session/'episodes'/episode
    directory.mkdir(parents=True)
    atomic_json(session/'owner.json',dict(session=name))
    atomic_json(session/'result.json',dict(session=name,finished_ns=123,clean=True))
    summary=dict(episode=episode,result=result,complete=result!='aborted',reason='控制连接关闭')
    files={'metadata.json':json.dumps(dict(dataset_kind='hardware_demonstration')),
           'summary.json':json.dumps(summary),
           'audit/events.jsonl':'','audit/raw.jsonl':'','media/manifest.jsonl':'','media/camera_samples.jsonl':''}
    manifest=[dict(file=k,bytes=len(v.encode()),sha256=hashlib.sha256(v.encode()).hexdigest()) for k,v in files.items()]
    with zipfile.ZipFile(directory/'bundle.zip','w') as bundle:
        for k,v in files.items():bundle.writestr(k,v)
        bundle.writestr('bundle_manifest.json',json.dumps(manifest))
    atomic_json(directory/'summary.json',summary)
    item=dict(episode=episode,phase='pending',bundle_sha256=digest_file(directory/'bundle.zip'))
    atomic_json(directory/'transfer.json',item)
    return directory,item


@pytest.fixture
def recovery(tmp_path):
    directory,item=saved_session(tmp_path/'sessions')
    catalogue=RetainedEpisodes(tmp_path/'sessions',recovery_id='test')
    server=start_server(catalogue,0,bytes_per_second=16*1024*1024)
    worker=TransferWorker(f'http://127.0.0.1:{server.server_port}',tmp_path/'pc')
    try:yield catalogue,worker,directory,item
    finally:worker.close();server.shutdown();server.server_close()


def test_finished_aborted_session_survives_restart_and_real_verified_receipt(recovery):
    catalogue,worker,directory,item=recovery
    before={name:(directory/name).read_bytes() for name in ('summary.json','bundle.zip')}
    listing=worker.get('/episodes')
    assert listing['current']['recovery_only'] and listing['current']['recovery_id']=='test'
    assert listing['episodes'][0]['retained_session']==SESSION
    assert listing['episodes'][0]['summary']['result']=='aborted'
    worker.start()
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:
        status=worker.snapshot()['episodes']
        if status and status[0]['phase']=='verified':break
        time.sleep(.01)
    assert status[0]['phase']=='verified' and status[0]['remote_verified']
    assert json.loads((directory/'transfer.json').read_text())['phase']=='verified'
    assert json.loads((worker.root/EPISODE/'summary.json').read_text())['complete'] is False
    assert {name:(directory/name).read_bytes() for name in before}==before
    assert RetainedEpisodes(catalogue.root).listing()['episodes'][0]['phase']=='verified'


@pytest.mark.parametrize('damage',['unsealed','damaged_json','wrong_summary','wrong_owner','unfinished','metadata_array'])
def test_bad_session_isolated_from_good_finished_session(tmp_path,damage):
    root=tmp_path/'sessions';bad,item=saved_session(root)
    good,_=saved_session(root,'20261009-205722-11d544a7','20261009T130739Z_fixture1','success')
    if damage=='unsealed':atomic_json(bad/'transfer.json',dict(item,bundle_sha256=None))
    if damage=='damaged_json':(bad/'transfer.json').write_text('{')
    if damage=='wrong_summary':atomic_json(bad/'summary.json',dict(episode='different'))
    if damage=='metadata_array':atomic_json(bad/'transfer.json',[])
    if damage=='wrong_owner':atomic_json(bad.parent.parent/'owner.json',dict(session='different'))
    if damage=='unfinished':(bad.parent.parent/'result.json').unlink()
    catalogue=RetainedEpisodes(root);listing=catalogue.listing()
    assert [x['episode'] for x in listing['episodes']]==[good.name]
    with pytest.raises(ValueError):catalogue.directory(EPISODE)


def test_duplicate_id_rejected_even_when_one_copy_is_unsealed(recovery):
    catalogue,worker,directory,item=recovery
    other,_=saved_session(catalogue.root,'20261009-205722-11d544a7')
    (other/'transfer.json').unlink()
    listing=worker.get('/episodes')
    assert not listing['episodes']
    assert listing['catalogue_errors'][0]['error']=='Duplicate retained episode identifier'
    assert worker.one(item)['phase']=='failed'
    assert not (worker.root/EPISODE/'bundle.zip').exists()
    assert json.loads((directory/'transfer.json').read_text())['phase']=='pending'


def test_checksum_error_explicit_retry_and_wrong_receipt_preserve_source(recovery):
    catalogue,worker,directory,item=recovery
    original=(directory/'bundle.zip').read_bytes()
    (directory/'bundle.zip').write_bytes(b'bad transport fixture')
    first=worker.one(item)
    assert first['phase']=='failed' and first['failed_step']=='download'
    assert json.loads((directory/'transfer.json').read_text())['phase']=='pending'
    with pytest.raises(ValueError):catalogue.transferred(EPISODE,'0'*64)
    (directory/'bundle.zip').write_bytes(original)
    worker.retry(EPISODE)
    second=worker.one(item)
    assert second['phase']=='verified' and second['attempt_errors']
    assert list((worker.root/EPISODE).glob('bundle.failed-*.part'))
    assert (directory/'bundle.zip').read_bytes()==original


def test_path_escape_and_selected_episode_do_not_serve_other_files(tmp_path):
    root=tmp_path/'sessions';directory,_=saved_session(root)
    catalogue=RetainedEpisodes(root,episode=EPISODE)
    saved_session(root,'20261009-205722-11d544a7','20261009T130739Z_fixture1')
    assert len(catalogue.listing()['episodes'])==1
    for name in ('../outside','/etc/passwd','a\\outside'):
        with pytest.raises(ValueError):catalogue.directory(name)
    outside=tmp_path/'outside';outside.write_text('private')
    bundle=directory/'bundle.zip';bundle.unlink()
    try:bundle.symlink_to(outside)
    except OSError:pytest.skip('Symlink creation unavailable on this host')
    assert not catalogue.listing()['episodes']
    assert catalogue.listing()['catalogue_errors']


def test_recovery_pc_never_observes_or_forwards_control(monkeypatch,tmp_path):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from starlette.websockets import WebSocketDisconnect
    from e6_phosphobot import pc_bridge
    from e6_phosphobot import episode_transfer
    calls=[]
    def forbidden(*a,**kw):raise AssertionError('Device control/observe connection attempted')
    monkeypatch.setattr(pc_bridge,'connect',forbidden)
    monkeypatch.setattr(episode_transfer.TransferWorker,'start',lambda self:calls.append('media_start'))
    monkeypatch.setattr(episode_transfer.TransferWorker,'close',lambda self:calls.append('media_close'))
    app=FastAPI()
    pc_bridge.install_hardware(app,'ws://127.0.0.1:8776',episodes_dir=tmp_path,recordings_only=True)
    with TestClient(app) as client:
        assert client.get('/e6/capabilities').json()['enabled'] is False
        assert client.get('/e6/episodes').json()['recordings_only']
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect('/e6/control'):pass
        for path in ('/move/init','/joints/write','/move/absolute'):
            assert client.post(path,json={}).status_code==409
    assert calls==['media_start','media_close']


def test_media_only_tunnel_has_no_control_or_preview_transport():
    from e6_phosphobot.launcher import Tunnels
    closed=[]
    client=SimpleNamespace(close=lambda:closed.append(True))
    tunnels=Tunnels(lambda:client,media_only=True)
    assert tunnels.clients==[client] and tunnels.media is client
    assert not hasattr(tunnels,'control') and not hasattr(tunnels,'preview')
    tunnels.close()
    assert closed==[True]


def test_remote_recovery_uses_pinned_media_environment_without_device_entrypoint():
    from e6_phosphobot.recording_recovery import recovery_command
    command=recovery_command(dict(release='/release',gateway_dependencies='/deps',remote_base='/platform'),'owner',EPISODE)
    assert 'PYTHONPATH=/release:/deps' in command
    assert command[command.index('-m')+1]=='e6_phosphobot.episode_http'
    assert command[command.index('--sessions-root')+1]=='/platform/sessions'
    assert command[-2:]==['--episode',EPISODE]
    assert '--stage' not in command and '--rootfs' not in command
