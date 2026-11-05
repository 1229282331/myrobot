import base64
import hashlib
import json
import sys
from pathlib import Path
from unittest.mock import patch
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.archive import ArchiveStore
from e6_observer.protocol import encode_wire,decode_wire
from e6_teleop.observation_recording import ObservationRecorder
from test_observation import frame


@pytest.mark.parametrize('failure',[None,'transfer'])
def test_service_native_metadata_recording_and_explicit_transfer(tmp_path,failure):
    import asyncio
    import time
    from fastapi.testclient import TestClient
    from e6_teleop.observation_service import make_app
    store=ArchiveStore(tmp_path/'remote')
    class Connection:
        async def __aenter__(self):return self
        async def __aexit__(self,*args):pass
        async def send(self,value):self.probe=json.loads(value)
        def __aiter__(self):return self.messages()
        async def messages(self):
            if hasattr(self,'probe'):
                for _ in range(5):yield json.dumps(dict(clock_probe=self.probe['nonce'],x5_monotonic_ns=time.perf_counter_ns()))
            for i in range(100):
                await asyncio.sleep(.05)
                f=native(i);store.capture(f)
                f['capabilities']['deferred_media']=True;f['transport_profile']='latest-preview-deferred-originals'
                for n,key in (('rgb','jpeg'),('depth','uint16_raw')):
                    f['sources'][n].pop(key)
                    f['sources'][n].pop('preview_jpeg',None)
                yield json.dumps(f)
    async def begin(gateway,instance,session):store.begin(session)
    async def transfer(gateway,instance,recorder):
        store.seal(recorder.directory.name)
        await asyncio.sleep(.15)
        if failure:raise ConnectionError('injected native archive interruption')
        for spec in recorder.specs.values():
            entry,data=store.fetch(recorder.directory.name,spec['source'],spec['id'])
            recorder.import_media(spec,data,entry)
        store.release(recorder.directory.name)
    try:
        with patch('e6_teleop.observation_service.websockets.connect',side_effect=lambda *args,**kw:Connection()),patch('e6_teleop.observation_service.begin_archive',side_effect=begin),patch('e6_teleop.observation_service.retrieve_archive',side_effect=transfer):
            with TestClient(make_app(tmp_path/'local',render=False)) as c:
                for _ in range(50):
                    if c.get('/api/state').json()['gateway_ready']:break
                    time.sleep(.02)
                assert c.post('/api/record/start',json={'task':'deferred'}).status_code==200
                time.sleep(.2)
                result=c.post('/api/record/finish',json={'result':'observed'})
                assert result.status_code==200 and result.json()['pending']
                assert c.post('/api/record/start',json={'task':'premature'}).status_code==409
                deadline=time.monotonic()+2
                while True:
                    state=c.get('/api/state').json()['recording']
                    if not state['pending'] or time.monotonic()>deadline:break
                    time.sleep(.02)
                assert not state['pending']
                episode=c.get('/api/episodes').json()[0]
                assert episode['complete']==(failure is None)
                if failure:assert episode['result']=='aborted'
                assert c.post('/api/export/'+episode['id']).status_code==400
    finally:store.close()


def native(seq=0):
    f=frame(seq)
    for n,key in (('rgb','jpeg'),('depth','uint16_raw')):
        s=f['sources'][n];data=base64.b64decode(s.pop(key))
        s[key]=data;s['original_sha256']=hashlib.sha256(data).hexdigest();s['original_bytes']=len(data)
        s['preview_jpeg']=f['sources']['rgb'].get('jpeg',b'preview')
    return f


def test_archive_selected_ids_are_immutable_and_verified(tmp_path):
    store=ArchiveStore(tmp_path/'x5')
    r=ObservationRecorder(tmp_path/'pc')
    name=r.start('wireless',{'media_storage':'x5-spool-then-verified-pc'})
    try:
        store.begin(name)
        for i in range(4):
            f=native(i);store.capture(f)
            for n,key in (('rgb','jpeg'),('depth','uint16_raw')):f['sources'][n].pop(key)
            r.sample(f)
        status=store.seal(name);assert status['unique_files']==4
        r.finish('observed')
        assert r.pending and not json.loads((r.directory/'summary.json').read_text(encoding='utf-8'))['complete']
        with pytest.raises(ValueError,match='传输'):r.start('premature',{})
        for spec in r.specs.values():
            entry,data=store.fetch(name,spec['source'],spec['id']);r.import_media(spec,data,entry)
        r.finalize_media();assert r.phase=='complete'
        rows=[json.loads(x) for x in (r.directory/'samples.jsonl').read_text(encoding='utf-8').splitlines()]
        assert rows[1]['sources']['rgb']['repeated'] and not rows[1]['sources']['rgb']['media_pending']
        assert all(x['sources']['depth']['sha256'] and x['input'] is None and x['effective_action'] is None for x in rows)
        with pytest.raises(ValueError):store.begin('other_1234')
        store.release(name);store.begin('other_1234');store.seal('other_1234')
    finally:store.close()


@pytest.mark.parametrize('failure',['disk','budget','queue'])
def test_archive_failures_are_visible_not_silently_dropped(tmp_path,failure):
    store=ArchiveStore(tmp_path,max_bytes=1 if failure=='budget' else 2**20)
    try:
        store.begin('session_1234')
        if failure=='disk':
            with patch.object(Path,'write_bytes',side_effect=OSError('injected disk full')):
                store.capture(native());store.queue.join()
        elif failure=='queue':
            import queue
            with patch.object(store.queue,'put_nowait',side_effect=queue.Full):store.capture(native())
        else:store.capture(native())
        assert store.status()['error'];assert not store.status()['active']
        assert not store.seal('session_1234')['complete']
    finally:store.close()


def test_archive_path_identity_checksum_and_interrupted_transfer(tmp_path):
    store=ArchiveStore(tmp_path/'x5');r=ObservationRecorder(tmp_path/'pc')
    name=r.start('incomplete',{'media_storage':'x5-spool-then-verified-pc'})
    try:
        with pytest.raises(ValueError):store.begin('../../escape')
        store.begin(name);f=native();store.capture(f);r.sample(f);store.seal(name)
        r.finish('observed');spec=next(iter(r.specs.values()))
        entry,data=store.fetch(name,spec['source'],spec['id'])
        with pytest.raises(ValueError,match='checksum'):r.import_media(spec,data+b'x',entry)
        with pytest.raises(ValueError):store.fetch('wrong_1234','rgb',0)
        with pytest.raises(ValueError):store.fetch(name,'../../escape',0)
        r.finalize_media('SSH archive interrupted')
        summary=json.loads((r.directory/'summary.json').read_text(encoding='utf-8'))
        assert not summary['complete'] and summary['result']=='aborted' and r.samples==1
    finally:store.close()


def test_preview_wire_is_separate_from_native_pixels_and_age():
    f=native();original=f['sources']['rgb']['jpeg']
    for n,key in (('rgb','jpeg'),('depth','uint16_raw')):f['sources'][n].pop(key)
    sent={};received={}
    result=decode_wire(encode_wire(f,sent),received)
    assert base64.b64decode(result['sources']['rgb']['preview_jpeg'])==original
    assert 'jpeg' not in result['sources']['rgb'] and 'uint16_raw' not in result['sources']['depth']
    assert result['sources']['rgb']['original_sha256']==hashlib.sha256(original).hexdigest()
    assert result['sources']['rgb']['x5_receive_monotonic_ns']==f['sources']['rgb']['x5_receive_monotonic_ns']
    assert decode_wire(encode_wire(f,sent),received)==result


def test_control_sampling_owner_release_does_not_leak_or_clear_existing_fault():
    import importlib,threading,types
    from collections import deque
    with patch.dict(sys.modules,{'fcntl':types.SimpleNamespace()}):
        Hub=importlib.import_module('e6_observer.runtime').Hub
    hub=Hub.__new__(Hub);hub.lock=threading.Lock();hub.control_sampling_owner=None
    hub.control_sampling=False;hub.control_sampling_started_ns=None;hub.control_backlog=deque()
    hub.stats={'fatal':'prior stream failure'}
    first=object();other=object();hub.begin_control_sampling(first)
    hub.control_backlog.append({'tick':1})
    with pytest.raises(ValueError,match='consumer'):hub.begin_control_sampling(other)
    hub.end_control_sampling(other)
    assert hub.control_sampling and list(hub.control_backlog)==[{'tick':1}]
    hub.end_control_sampling(first)
    assert not hub.control_sampling and hub.control_sampling_owner is None and not hub.control_backlog
    assert hub.stats['fatal']=='prior stream failure'
    hub.begin_control_sampling(other);assert hub.control_sampling and not hub.control_backlog


def test_cancelled_transfer_waits_for_owned_write_before_return(tmp_path):
    import asyncio
    import threading
    import time
    import zstandard
    from e6_teleop.observation_archive import retrieve_archive
    r=ObservationRecorder(tmp_path)
    name=r.start('shutdown',{'media_storage':'x5-spool-then-verified-pc'})
    f=native();f['archive_session']=name;r.sample(f);r.finish('observed')
    spec=next(iter(r.specs.values()));data=f['sources'][spec['source']]['jpeg']
    payload=zstandard.ZstdCompressor(level=1).compress(data)
    entry=dict(source=spec['source'],id=spec['id'],sha256=spec['sha256'],x5_written_monotonic_ns=1)
    began=threading.Event();ended=threading.Event();original=r.import_media
    def write(*args):
        began.set();time.sleep(.1);original(*args);ended.set()
    class Connection:
        async def __aenter__(self):return self
        async def __aexit__(self,*args):pass
        async def send(self,value):self.operation=json.loads(value)['operation'];self.sent=False
        async def recv(self):
            if self.operation=='seal':return json.dumps(dict(ok=True,archive={}))
            if not self.sent:
                self.sent=True
                return json.dumps(dict(ok=True,entry=entry,wire_bytes=len(payload),wire_sha256=hashlib.sha256(payload).hexdigest()))
            return payload
    async def run():
        with patch('e6_teleop.observation_archive.websockets.connect',return_value=Connection()),patch.object(r,'import_media',side_effect=write):
            task=asyncio.create_task(retrieve_archive('ws://127.0.0.1:1','one',r))
            while not began.is_set():await asyncio.sleep(.005)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):await task
            assert ended.is_set() and len(r.verified)==1
        r.finalize_media('shutdown')
        assert not json.loads((r.directory/'summary.json').read_text(encoding='utf-8'))['complete']
    asyncio.run(run())
