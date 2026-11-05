import asyncio
import hashlib
import io
import json
from pathlib import Path
import threading
import zipfile

from fastapi import APIRouter, FastAPI, HTTPException, Request
from fastapi.testclient import TestClient
import pytest

from e6_phosphobot import problem_reports as reports
from e6_phosphobot.pc_bridge import StateCache


SESSION = '20261009-053012-87e1b7e0'
EPISODE = '20261008T213118Z_0a0e5077'


def put(root, name, value):
    path = root/name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf8')
    return path


def unpack(body):
    with zipfile.ZipFile(io.BytesIO(body)) as archive:
        return {name: archive.read(name) for name in archive.namelist()}


def service(root, snapshot=lambda: dict(state=None, fresh=False)):
    app, router = FastAPI(), APIRouter()
    reports.install_problem_reports(router, root, snapshot)
    app.include_router(router)
    return TestClient(app), router


def test_offline_bundle_download_checksums_and_immutable_originals(tmp_path):
    source = put(tmp_path, f'runtime/sessions/{SESSION}/result.json', {'exit_code': 1, 'failure': 'disconnect'})
    original = source.read_bytes()
    client, _ = service(tmp_path)
    response = client.post('/e6/diagnostics/problem')
    assert response.status_code == 200
    receipt = response.json()
    downloaded = client.get(receipt['url'])
    assert 'attachment' in downloaded.headers['content-disposition']
    assert len(downloaded.content) == receipt['bytes']
    assert hashlib.sha256(downloaded.content).hexdigest() == receipt['sha256']
    files = unpack(downloaded.content)
    manifest = json.loads(files['manifest.json'])
    assert json.loads(files['current-state.json']) == {'state': None, 'fresh': False}
    assert manifest['device_access'] is False and manifest['original_records_modified'] is False
    assert receipt['omissions'] == len(manifest['omissions'])
    for entry in manifest['entries']:
        assert hashlib.sha256(files[entry['file']]).hexdigest() == entry['sha256']
        assert len(files[entry['file']]) == entry['bytes']
    assert source.read_bytes() == original
    assert not list((tmp_path/'runtime/diagnostics').glob('*.part'))


def test_credentials_and_unstructured_logs_excluded(tmp_path):
    session = f'runtime/sessions/{SESSION}'
    put(tmp_path, session+'/deployment.json', {'stage': 'shadow', 'password': 'DEPLOYSECRET',
        'source_manifest': {'sha256': 'abc'}, 'ssh': {'password': 'SSHSECRET'}})
    put(tmp_path, session+'/result.json', {'token': 'SECRET1', 'nested': {'api-key': 'SECRET2'},
        'message': 'password=SECRET3 https://user:SECRET4@localhost bearer SECRET5',
        'key': '-----BEGIN OPENSSH PRIVATE KEY-----\nSECRET6\n-----END OPENSSH PRIVATE KEY-----'})
    (tmp_path/session/'ssh.log').write_text('SECRET7\n-----END OPENSSH PRIVATE KEY-----')
    (tmp_path/session/'lifecycle.jsonl').write_text(json.dumps({'event': 'exit', 'password': 'SECRET8'})+'\nSECRET9\n')
    put(tmp_path, 'auto.key', {'data': 'SECRET10'})
    result = reports.build_report(tmp_path, {'state': {'authorization': 'SECRET11'}})
    files = unpack((tmp_path/'runtime/diagnostics'/result['name']).read_bytes())
    contents = b'\n'.join(files.values())
    assert b'SECRET' not in contents
    assert not any(name.endswith(('.log', '.key')) for name in files)
    deployment = json.loads(files[session+'/deployment.json'])
    assert deployment == {'stage': 'shadow', 'source_manifest': {'sha256': 'abc'}}
    event = json.loads(files[session+'/lifecycle.jsonl'])
    assert event == {'event': 'exit', 'password': '[REDACTED]'}
    assert any(x['reason']=='invalid_jsonl_records' for x in json.loads(files['manifest.json'])['omissions'])


def test_tail_is_complete_jsonl_latest_scope_and_limits(tmp_path, monkeypatch):
    monkeypatch.setattr(reports, 'MAX_FILE_BYTES', 100)
    monkeypatch.setattr(reports, 'MAX_TOTAL_BYTES', 200)
    for date in ('20261007', '20261008', '20261009'):
        path = put(tmp_path, f'runtime/sessions/{date}-053012-87e1b7e0/result.json', {'a': 'x'*101})
        (path.parent/'lifecycle.jsonl').write_text('invalid'+('x'*100)+'\n'+json.dumps({'event': 'last', 'n': date})+'\n')
    result = reports.build_report(tmp_path, {})
    files = unpack((tmp_path/'runtime/diagnostics'/result['name']).read_bytes())
    manifest = json.loads(files.pop('manifest.json'))
    assert sum(map(len, files.values())) <= 200
    assert all(len(body)<=100 for body in files.values())
    assert not any('20261007' in x['file'] for x in manifest['entries']+manifest['omissions'])
    tails = [e for e in manifest['entries'] if e['file'].endswith('.jsonl')]
    assert len(tails)==2 and all(e['truncated'] for e in tails)
    assert all(json.loads(files[e['file']])['event']=='last' for e in tails)
    assert any(x['reason']=='file_byte_budget' for x in manifest['omissions'])


def test_damaged_records_are_not_copied(tmp_path):
    path = put(tmp_path, f'runtime/episodes/{EPISODE}/metadata.json', {})
    path.write_text('not json')
    put(tmp_path, f'runtime/episodes/{EPISODE}/summary.json', {'outcome': 'aborted'})
    result = reports.build_report(tmp_path, {})
    files = unpack((tmp_path/'runtime/diagnostics'/result['name']).read_bytes())
    assert not any(name.endswith('metadata.json') for name in files)
    assert json.loads(files[f'runtime/episodes/{EPISODE}/summary.json'])['outcome']=='aborted'


def test_same_origin_and_download_names(tmp_path):
    client, _ = service(tmp_path)
    assert client.post('/e6/diagnostics/problem', headers={'origin': 'https://other.example'}).status_code==403
    assert not (tmp_path/'runtime').exists()
    assert client.post('/e6/diagnostics/problem', headers={'origin': 'http://testserver'}).status_code==200
    for name in ('auto.key', 'problem-'+'a'*32+'.part', 'problem-'+'a'*32+'.zip'):
        assert client.get('/e6/diagnostics/problem/'+name).status_code==404


def test_disk_failure_preserves_previous_bundle_and_releases_lock(tmp_path, monkeypatch):
    client, _ = service(tmp_path)
    receipt = client.post('/e6/diagnostics/problem').json()
    previous = client.get(receipt['url']).content
    original = Path.replace
    def fail(*args):
        raise OSError('disk full')
    monkeypatch.setattr(Path, 'replace', fail)
    for _ in range(2):
        assert client.post('/e6/diagnostics/problem').status_code==507
    assert not list((tmp_path/'runtime/diagnostics').glob('*.part'))
    assert client.get(receipt['url']).content==previous
    monkeypatch.setattr(Path, 'replace', original)
    assert client.post('/e6/diagnostics/problem').status_code==200


def test_external_symlink_cannot_write_or_download(tmp_path):
    root, outside = tmp_path/'workstation', tmp_path/'outside'
    root.mkdir(); outside.mkdir()
    try:
        (root/'runtime').symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip('Symlinks require Windows privilege')
    client, _ = service(root)
    assert client.post('/e6/diagnostics/problem').status_code==507
    assert not list(outside.iterdir())
    (outside/'diagnostics').mkdir()
    (outside/'diagnostics'/('problem-'+'a'*32+'.zip')).write_bytes(b'private')
    assert client.get('/e6/diagnostics/problem/problem-'+'a'*32+'.zip').status_code==404


def test_cancelled_request_keeps_single_writer_and_event_loop_responsive(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def slow(*args):
        entered.set()
        assert release.wait(3)
        return {'saved': True}
    monkeypatch.setattr(reports, 'build_report', slow)
    _, router = service(tmp_path)
    create = router.routes[0].endpoint
    request = Request({'type': 'http', 'method':'POST', 'path':'/', 'headers': [], 'scheme':'http', 'server':('testserver',80)})
    async def run():
        first = asyncio.create_task(create(request))
        try:
            assert await asyncio.to_thread(entered.wait, 1)
            await asyncio.wait_for(asyncio.sleep(.01), .2)
            first.cancel()
            with pytest.raises(asyncio.CancelledError):
                await first
            with pytest.raises(HTTPException) as error:
                await create(request)
            assert error.value.status_code==409
        finally:
            release.set()
        for _ in range(100):
            await asyncio.sleep(.01)
            try:
                assert await create(request)=={'saved':True}
                return
            except HTTPException as error:
                assert error.status_code==409
        pytest.fail('Writer lock remained held after completion')
    asyncio.run(run())


def test_cached_state_retains_local_clock_and_staleness(monkeypatch):
    from e6_phosphobot import pc_bridge
    cache = StateCache()
    assert cache.diagnostics()==dict(state=None, pc_received_ns=None, pc_age_ms=None, fresh=False)
    monkeypatch.setattr(pc_bridge.time, 'perf_counter_ns', lambda: 100)
    state = {'kind':'state', 'actual':{'received_ns':999999999999}}
    cache.update(state)
    assert cache.diagnostics()['fresh'] is True
    monkeypatch.setattr(pc_bridge.time, 'perf_counter_ns', lambda: 600000100)
    old = cache.diagnostics()
    assert old['fresh'] is False and old['pc_age_ms']==600
    assert old['state'] is state and old['pc_received_ns']==100
    with pytest.raises(HTTPException):
        cache.current()
