"""Real loopback media protocol and temporary files; no robot or camera sockets."""
import json
import time
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest
from e6_phosphobot import episode_transfer as module
from e6_phosphobot.camera_archive import atomic_json
from e6_phosphobot.episodes import Episodes
from e6_phosphobot.episode_http import start_server


@pytest.fixture
def transfer(tmp_path):
    episode='20261008T000000Z_fixture'
    remote=tmp_path/'remote'/episode;remote.mkdir(parents=True)
    files={'metadata.json':json.dumps(dict(dataset_kind='hardware_shadow')),
           'summary.json':json.dumps(dict(result='success')),
           'audit/events.jsonl':'','audit/raw.jsonl':'',
           'media/manifest.jsonl':'','media/camera_samples.jsonl':''}
    import hashlib
    manifest=[dict(file=name,bytes=len(value.encode()),sha256=hashlib.sha256(value.encode()).hexdigest())
              for name,value in files.items()]
    with zipfile.ZipFile(remote/'bundle.zip','w') as bundle:
        for name,value in files.items():bundle.writestr(name,value)
        bundle.writestr('bundle_manifest.json',json.dumps(manifest))
    digest=module.digest_file(remote/'bundle.zip')
    item=dict(episode=episode,bundle_sha256=digest,phase='pending')
    atomic_json(remote/'transfer.json',item)
    atomic_json(remote/'summary.json',dict(result='success'))
    episodes=Episodes(tmp_path/'remote',tmp_path/'camera',SimpleNamespace(),'shadow')
    episodes.value.update(phase='transfer_pending',episode=episode,bundle_sha256=digest,result='success')
    server=start_server(episodes,0)
    worker=module.TransferWorker(f'http://127.0.0.1:{server.server_port}',tmp_path/'pc')
    try:yield worker,episodes,item
    finally:worker.close();server.shutdown();server.server_close();episodes.close()


@pytest.mark.parametrize('phase',['transferring','verified'])
def test_local_status_write_failure_clears_activity_and_explicit_retry_recovers(transfer,monkeypatch,phase):
    worker,episodes,item=transfer
    real=module.atomic_json
    def fail(path,value):
        if path.name=='pc_transfer.json' and value['phase']==phase:
            raise OSError('injected PC status disk failure')
        return real(path,value)
    monkeypatch.setattr(module,'atomic_json',fail)
    result=worker.one(item)
    assert result['phase']=='failed' and 'injected PC status disk failure' in result['error']
    assert worker.current is None
    assert worker.snapshot()['episodes'][0]['phase']=='failed'
    expected='transfer_pending' if phase=='transferring' else 'complete'
    assert episodes.state()['phase']==expected
    monkeypatch.setattr(module,'atomic_json',real)
    worker.retry(item['episode'])
    recovered=worker.one(item)
    assert recovered['phase']=='verified' and worker.current is None
    assert recovered['attempt_errors'][0]['error']==result['error']
    assert episodes.state()['phase']=='complete'
    assert module.digest_file(worker.root/item['episode']/'bundle.zip')==item['bundle_sha256']


def test_unwritable_first_status_still_visible_on_formal_pc_route_and_retry(transfer):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from e6_phosphobot.pc_bridge import install_hardware
    worker,episodes,item=transfer
    directory=worker.root/item['episode'];directory.mkdir()
    obstruction=directory/'pc_transfer.tmp';obstruction.mkdir()
    result=worker.one(item)
    assert result['phase']=='failed' and worker.current is None
    assert not (directory/'pc_transfer.json').exists()
    # The formal API must retain an item and retry even when no status file
    # could be created. No application lifespan or device adapter is started.
    from unittest.mock import patch
    with patch.object(module,'TransferWorker',return_value=worker):
        app=FastAPI();install_hardware(app,'ws://127.0.0.1:8776',episodes_dir=worker.root)
    client=TestClient(app)
    view=client.get('/e6/episodes').json()
    assert view['transferring'] is None and view['episodes'][0]['phase']=='failed'
    assert view['episodes'][0]['status_saved'] is False
    assert episodes.state()['phase']=='transfer_pending'
    obstruction.rmdir()
    assert client.post(f"/e6/episodes/{item['episode']}/retry").json()['queued']
    worker.start()
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:
        view=client.get('/e6/episodes').json()
        if view['episodes'][0]['phase']=='verified':break
        time.sleep(.01)
    assert view['episodes'][0]['phase']=='verified',view
    assert view['episodes'][0]['attempt_errors']
    assert episodes.state()['phase']=='complete'


def test_receipt_http_error_retains_remote_reason_and_reuses_original_bundle(transfer):
    worker,episodes,item=transfer
    obstruction=episodes.root/item['episode']/'transfer.tmp';obstruction.mkdir()
    result=worker.one(item)
    assert result['phase']=='failed' and result['failed_step']=='receipt'
    assert 'HTTP 409' in result['error'] and 'transfer.tmp' in result['error']
    assert episodes.state()['phase']=='transfer_pending' and worker.current is None
    bundle=worker.root/item['episode']/'bundle.zip';before=bundle.stat().st_mtime_ns
    obstruction.rmdir()
    recovered=worker.one(item)
    assert recovered['phase']=='verified' and recovered['attempt_errors']
    assert bundle.stat().st_mtime_ns==before and module.digest_file(bundle)==item['bundle_sha256']
    assert episodes.state()['phase']=='complete'


def test_truncated_http_error_body_does_not_replace_original_failure():
    from http.client import IncompleteRead
    from urllib.error import HTTPError
    import io
    class Truncated(io.BytesIO):
        def read(self,*args):raise IncompleteRead(b'{"error":')
    error=HTTPError('http://127.0.0.1/verified',409,'Conflict',{},Truncated())
    assert module.TransferWorker.failure_text(error)=='HTTP Error 409: Conflict'


def test_transfer_progress_is_memory_only_until_verified_receipt(transfer,monkeypatch):
    worker,episodes,item=transfer
    real_open=module.urlopen;real_write=module.atomic_json
    snapshots=[];writes=[]
    class ObservedResponse:
        def __init__(self,response):self.response=response;self.headers=response.headers
        def __enter__(self):return self
        def __exit__(self,*args):self.response.close()
        def read(self,size):
            snapshot=worker.snapshot();snapshots.append(snapshot)
            # Isolate the snapshot from internal progress, as the HTTP reader
            # may retain or mutate its own serialized copy.
            snapshot['episodes'][0]['progress']['received_bytes']=-1
            time.sleep(.26)
            return self.response.read(min(size,512))
    def open_response(url,*args,**kwargs):
        response=real_open(url,*args,**kwargs)
        if isinstance(url,str) and url.endswith('/bundle'):return ObservedResponse(response)
        return response
    def write(path,value):
        if path.name=='pc_transfer.json':writes.append(value['phase'])
        return real_write(path,value)
    monkeypatch.setattr(module,'urlopen',open_response)
    monkeypatch.setattr(module,'atomic_json',write)
    real_publish=worker.publish_progress;published=[]
    def publish(status,stage,received,total):
        real_publish(status,stage,received,total)
        published.append(worker.snapshot()['episodes'][0])
        assert episodes.state()['phase']=='transfer_pending'
    monkeypatch.setattr(worker,'publish_progress',publish)
    result=worker.one(item)
    assert result['phase']=='verified',result
    assert writes==['transferring','verified']
    assert snapshots and all(row['phase']=='transferring' for row in published)
    received=[row['progress']['received_bytes'] for row in published]
    assert received==sorted(received) and len(set(received))>=3
    assert all(row['progress']['total_bytes']==received[-1] for row in published)
    assert {row['progress']['stage'] for row in published}=={'receiving','checksum','verification','receipt'}
    assert result['progress']==dict(stage='complete',received_bytes=received[-1],total_bytes=received[-1])
