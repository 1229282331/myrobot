import json
import threading
import time
from types import SimpleNamespace

import pytest

from e6_phosphobot.camera_restart import CameraRestart
from e6_phosphobot.gateway import NativeExecutor
from e6_phosphobot.latest_json_files import LatestJsonFiles


def wait_for(check):
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        if check():return
        time.sleep(.005)
    raise AssertionError('Background status not published')


def test_blocked_io_never_blocks_control_reads_or_refreshes_source_age(tmp_path):
    path=tmp_path/'health.json';now=[1_000_000_000]
    entered=threading.Event();release=threading.Event();calls=[]
    source=dict(serial='AY4A1520111',rgb_received_ns=now[0],frame_id=3)
    def load(_):
        calls.append(1)
        if len(calls)>1:
            entered.set();assert release.wait(3)
        return source.copy()
    cache=LatestJsonFiles([path],period=.001,load=load)
    e=NativeExecutor('unused','shadow',path,clock=lambda:now[0]);e.camera_io=cache
    cache.start()
    try:
        assert entered.wait(2)
        completed=threading.Event();answer=[]
        def get():answer.append(e.camera());completed.set()
        t=threading.Thread(target=get);t.start()
        assert completed.wait(.5)  # The file reader is still deliberately blocked.
        t.join();assert answer[0]['valid'] and answer[0]['frame_id']==3
        now[0]+=500_000_001
        assert not e.camera()['valid'] and e.camera()['age_ms']>500
    finally:release.set();cache.close()
    assert not cache.thread.is_alive()


@pytest.mark.parametrize('content',['missing','[]','{','x'*65537],
                         ids=['missing','array','malformed','oversized'])
def test_unreadable_or_malformed_file_replaces_old_valid_snapshot(tmp_path,content):
    path=tmp_path/'health.json';path.write_text('{"frame_id":1}')
    first_done=threading.Event();next_read=threading.Event();reads=[]
    def load(p):
        if reads:
            first_done.set();assert next_read.wait(3)
        reads.append(1)
        return LatestJsonFiles._load(p)
    cache=LatestJsonFiles([path],period=.002,load=load);cache.start()
    try:
        # Replace only after the first handle closes. Windows does not allow
        # unlinking a file concurrently held open by the background reader.
        assert first_done.wait(2)
        assert cache.read(path)['frame_id']==1
        if content=='missing':path.unlink()
        else:path.write_text(content)
        next_read.set()
        wait_for(lambda:cache.values[path][1] is not None)
        with pytest.raises(OSError):cache.read(path)
    finally:next_read.set();cache.close()


def test_cached_restart_keeps_request_identity_and_does_not_read_file(tmp_path,monkeypatch):
    now=[1_000_000_000];rows={}
    c=CameraRestart(tmp_path/'health.json','one',clock=lambda:now[0],status_reader=lambda p:rows[p])
    # No cached status is equivalent to no supervisor response, never approval.
    assert c.status()['phase']=='idle'
    c.request();identity=c.pending
    rows[c.status_path]=dict(session='one',id='other',phase='complete')
    assert c.status()['phase']=='requested'
    rows[c.status_path]=dict(session='one',id=identity,phase='complete')
    monkeypatch.setattr(type(c.status_path),'read_text',lambda *a,**kw:pytest.fail('Control file read'))
    assert c.status()['phase']=='complete'


def test_executor_owns_monitor_lifecycle_and_cached_camera_reads(tmp_path,monkeypatch):
    path=tmp_path/'health.json';now=time.perf_counter_ns()
    path.write_text(json.dumps(dict(serial='AY4A1520111',rgb_received_ns=now)))
    e=NativeExecutor('unused','shadow',path,clock=lambda:now,camera_session='one')
    calls=[]
    e.reader=SimpleNamespace(start=lambda:calls.append('start'),close=lambda:calls.append('close'))
    e.start()
    try:
        wait_for(lambda:e.camera()['valid'])
        assert e.camera_restart.status_reader is not None
        monkeypatch.setattr(type(path),'read_text',lambda *a,**kw:pytest.fail('Synchronous file read'))
        assert e.camera()['valid'] and e.camera_restart.status()['phase']=='idle'
    finally:e.close()
    assert calls==['start','close'] and not e.camera_io.thread.is_alive()
