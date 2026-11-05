import base64
import codecs
import json
import logging
import threading
import time
from pathlib import Path
from urllib.request import urlopen
from types import SimpleNamespace
import pytest
from fastapi import APIRouter,FastAPI
from fastapi.testclient import TestClient
from e6_phosphobot.operator_logs import AsyncLog,start_logging,audit_message
from e6_phosphobot.log_sources import LogSources,CHUNK_BYTES
from e6_phosphobot.log_api import install_log_api
from e6_phosphobot.episode_http import start_server


def test_native_text_transitions_traceback_and_debug(tmp_path):
    logger,sink=start_logging(tmp_path/'control.log','fixture.native')
    state={'state':'暂挂','reason':'边界','ready':False,'gripper':{'phase':'uninitialized'}}
    for _ in range(30):audit_message(logger,{'event':'cycle','state':state})
    for _ in range(20):audit_message(logger,{'event':'input','data':{'test':'no info flood'}})
    try:raise ValueError('中文异常\noriginal cause')
    except ValueError:logger.exception('Operation failed')
    logger.removeHandler(sink);sink.close()
    text=sink.path.read_text(encoding='utf-8')
    assert text.count('Control status / ready / reason')==1
    assert 'no info flood' not in text
    assert 'Traceback (most recent call last)' in text and 'ValueError: 中文异常\noriginal cause' in text
    assert 'INFO' in text and 'ERROR' in text and 'fixture.native' in text


def test_blocked_file_does_not_block_producer_or_shutdown(tmp_path):
    started=threading.Event();release=threading.Event()
    class Slow(AsyncLog):
        def _write(self,text):started.set();release.wait(5)
    sink=Slow(tmp_path/'slow',capacity=2)
    record=logging.LogRecord('test',logging.INFO,'',0,'message',(),None)
    sink.emit(record);assert started.wait(1)
    start=time.monotonic()
    for _ in range(2000):sink.emit(record)
    assert time.monotonic()-start<.2
    assert sink.queue.qsize()==2 and sink.dropped>=1998
    release.set();sink.close();assert not sink.thread.is_alive()


def test_gripper_attachment_and_fault_details_are_visible_without_frame_flood(tmp_path):
    logger,sink=start_logging(tmp_path/'control.log','fixture.gripper')
    audit_message(logger,dict(event='gripper_connecting',stroke={'closed_raw':12,'open_raw':255}))
    audit_message(logger,dict(event='gripper_connected',actual_raw=12,position_command_sent=False))
    audit_message(logger,dict(event='gripper_fault',reason='original stalled reason'))
    for _ in range(20):audit_message(logger,dict(event='gripper_observation',feedback={'raw':12}))
    logger.removeHandler(sink);sink.close()
    text=sink.path.read_text(encoding='utf-8')
    assert 'closed_raw' in text and 'position_command_sent=False' in text
    assert 'original stalled reason' in text and 'gripper_observation' not in text


def test_write_failure_isolated(tmp_path):
    blocker=tmp_path/'file';blocker.write_text('x')
    logger,sink=start_logging(blocker/'bad.log','fixture.failure')
    logger.info('Cannot write');logger.removeHandler(sink);sink.close()
    assert sink.write_errors==1 and sink.error


def registry(tmp_path):
    source=LogSources('session');file=tmp_path/'control.log';file.touch()
    source.register('control',file,root=tmp_path)
    return source,file


def test_utf8_chunks_reconnect_rotation_copytruncate_and_snapshot(tmp_path):
    logs,file=registry(tmp_path)
    file.write_bytes(b'initial\n')
    data,headers=logs.read('control');assert data==b'initial\n'
    decoder=codecs.getincrementaldecoder('utf-8')()
    word='中文\nTraceback:\n  line 1\n'.encode()
    with file.open('ab') as out:out.write(word[:2])
    part,after=logs.read('control',headers['X-Log-Cursor']);assert decoder.decode(part)==''
    with file.open('ab') as out:out.write(word[2:])
    part,after=logs.read('control',after['X-Log-Cursor']);assert decoder.decode(part)=='中文\nTraceback:\n  line 1\n'
    assert logs.read('control',after['X-Log-Cursor'])[0]==b''
    file.rename(tmp_path/'control.old');file.write_bytes(b'rotated\n')
    part,after=logs.read('control',after['X-Log-Cursor']);assert after['X-Log-Reset']=='1' and part==b'rotated\n'
    # Rewritten to a larger size with the same inode: anchor detects lost history.
    file.write_bytes(b'x'*100)
    part,after=logs.read('control',after['X-Log-Cursor']);assert after['X-Log-Reset']=='1'
    reader,size=logs.download('control')
    with file.open('ab') as out:out.write(b'later')
    with reader:assert reader.read(size)==b'x'*100
    assert (tmp_path/'control.old').read_bytes()==b'initial\n'+word


def test_source_allowlist_and_bounded_initial_window(tmp_path):
    logs,file=registry(tmp_path);file.write_bytes(('中'*40000).encode())
    data,headers=logs.read('control');assert len(data)<=CHUNK_BYTES
    assert data.decode()=='中'*(len(data)//3)
    for name in ('../control.log','/etc/passwd','other','pc'):
        with pytest.raises(ValueError):logs.read(name)
    with pytest.raises(ValueError):logs.read('control','?'*1025)
    logs.register('servo',tmp_path.parent/'outside',root=tmp_path)
    with pytest.raises(ValueError):logs.path('servo')
    assert [x['id'] for x in logs.listing()['sources']]==['control','servo']


def test_media_and_pc_routes_forward_raw_text_only(tmp_path):
    remote,file=registry(tmp_path);file.write_bytes('中文\n  stack\n'.encode())
    server=start_server(SimpleNamespace(root=tmp_path/'episodes'),0,logs=remote)
    local=LogSources('pc-session');local_file=tmp_path/'pc.log';local_file.write_bytes(b'PC line\n')
    local.register('pc',local_file,root=tmp_path)
    app=FastAPI();router=APIRouter()
    install_log_api(router,f'http://127.0.0.1:{server.server_port}',local);app.include_router(router)
    try:
        with TestClient(app) as client:
            result=client.get('/e6/logs').json();assert result['remote_session']=='session'
            response=client.get('/e6/logs/control/stream')
            assert response.text=='中文\n  stack\n' and response.headers['x-log-reset']=='1'
            assert client.get('/e6/logs/control/stream',params={'cursor':response.headers['x-log-cursor']}).content==b''
            assert client.get('/e6/logs/control/download').content==file.read_bytes()
            assert client.get('/e6/logs/pc/download').text=='PC line\n'
            assert client.get('/e6/logs/unknown/download').status_code==404
            assert client.post('/e6/logs/control/stream').status_code==405
            assert client.get('/e6/logs/control/stream',params={'cursor':'x'*1025}).status_code==400
    finally:server.shutdown();server.server_close()
