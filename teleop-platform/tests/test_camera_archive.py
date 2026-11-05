import hashlib
import json
import time
import pytest
from e6_phosphobot.camera_archive import CameraArchive,atomic_json


def offer(archive,index=1):
    for stream,data in [('color',b'original-jpeg'),('depth',bytes([0,1,2,3]))]:
        archive.offer(dict(stream=stream,index=index,host_perf_counter_ns=time.perf_counter_ns(),
                           width=2,height=1,device_timestamp_us=123,uvc_pts_32=45),data)


def test_original_bytes_metadata_and_deduplication_survive_native_archive(tmp_path):
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='request1',op='start',session='episode_123'))
        offer(archive);archive.sample();archive.sample()
        archive.request(dict(id='request2',op='finish',session='episode_123'))
        status=archive.status()
        assert status['unique_files']==2 and status['sample_index']==2
        entry,data=archive.store.fetch('episode_123','depth',1)
        assert data==bytes([0,1,2,3]) and entry['sha256']==hashlib.sha256(data).hexdigest()
        assert entry['metadata']['metadata']['uvc_pts_32']==45
        assert entry['metadata']['metadata']['device_timestamp_us']==123
        samples=[json.loads(x) for x in (tmp_path/'media/episode_123/camera_samples.jsonl').read_text().splitlines()]
        assert all(r['sources']['rgb']['fresh'] for r in samples)
        assert samples[0]['sources']['depth']['original_bytes']==4
    finally:archive.close()


def test_interrupted_capture_is_retained_aborted_and_no_camera_actions_exist(tmp_path):
    archive=CameraArchive(tmp_path)
    atomic_json(archive.request_path,dict(id='request1',op='start',session='episode_123'))
    offer(archive);archive.start()
    deadline=time.monotonic()+2
    while archive.public_status['sample_index']==0 and time.monotonic()<deadline:time.sleep(.01)
    archive.close()
    status=json.loads(archive.status_path.read_text())
    assert status['phase']=='aborted' and status['unique_files']==2
    assert (tmp_path/'media/episode_123/manifest.jsonl').is_file()


def test_wrong_session_never_seals_active_recording(tmp_path):
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='request1',op='start',session='episode_123'))
        with pytest.raises(ValueError):archive.request(dict(id='request2',op='finish',session='episode_456'))
        assert archive.store.active
    finally:archive.close()


def test_source_receipt_is_preserved_separately_and_old_source_is_not_freshened(tmp_path):
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='start',op='start',session='receipt_123'))
        now=time.perf_counter_ns();old=now-250_000_000
        archive.offer(dict(stream='color',index=1,host_perf_counter_ns=now,
            source_received_perf_ns=old,capture_epoch='after-gap'),b'original')
        archive.sample()
        archive.request(dict(id='finish',op='finish',session='receipt_123'))
        row=json.loads((tmp_path/'media/receipt_123/camera_samples.jsonl').read_text())['sources']['rgb']
        assert row['x5_receive_monotonic_ns']==old and not row['fresh']
        assert row['metadata']['host_perf_counter_ns']==now and row['metadata']['capture_epoch']=='after-gap'
    finally:archive.close()


def test_media_budget_failure_isolated_from_frame_producer(tmp_path):
    archive=CameraArchive(tmp_path)
    try:
        archive.store.max_bytes=1
        archive.request(dict(id='request1',op='start',session='episode_123'))
        offer(archive)
        with pytest.raises(RuntimeError):archive.sample()
        # The independent camera callback remains usable after storage failure.
        offer(archive,2)
        assert archive.latest['rgb'][0]['index']==2
        assert 'budget' in archive.store.error
    finally:archive.close()


def test_camera_restart_cannot_reuse_an_archived_image_identity(tmp_path):
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='request1',op='start',session='episode_123'))
        offer(archive,5);archive.sample()
        offer(archive,1)
        with pytest.raises(RuntimeError,match='identity restarted'):archive.sample()
    finally:archive.close()
def test_windows_metadata_sharing_race_retains_old_until_atomic_commit(tmp_path,monkeypatch):
    import json
    from pathlib import Path
    from types import SimpleNamespace
    import e6_phosphobot.camera_archive as module
    target=tmp_path/'state.json';target.write_text('{"old":true}')
    original=Path.replace;attempts=[]
    monkeypatch.setattr(module,'os',SimpleNamespace(name='nt'))
    def replace(source,destination):
        attempts.append(source.read_bytes())
        if len(attempts)<3:
            assert json.loads(target.read_text())=={'old':True}
            error=PermissionError('sharing violation');error.winerror=32;raise error
        return original(source,destination)
    monkeypatch.setattr(Path,'replace',replace)
    module.atomic_json(target,{'new':True})
    assert len(attempts)==3 and len(set(attempts))==1
    assert json.loads(target.read_text())=={'new':True}


def test_persistent_metadata_denial_still_fails(tmp_path,monkeypatch):
    import pytest
    from pathlib import Path
    from types import SimpleNamespace
    import e6_phosphobot.camera_archive as module
    monkeypatch.setattr(module,'os',SimpleNamespace(name='nt'))
    def denied(*args):
        error=PermissionError('permanent denial');error.winerror=5;raise error
    monkeypatch.setattr(Path,'replace',denied)
    with pytest.raises(PermissionError,match='permanent denial'):
        module.atomic_json(tmp_path/'state.json',{})


def test_archive_diagnostics_distinguish_sample_and_status_write_without_retiming_sources(tmp_path):
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='start',op='start',session='timing_123'))
        offer(archive);source_ns=archive.latest['rgb'][0]['host_perf_counter_ns']
        archive.sample()
        # Explicit synthetic timing samples; the recording timestamps stay original.
        for i in range(20):archive.timing('status_write',i*200_000_000,i*200_000_000+150_000_000)
        stats=archive.status()['timing']
        assert stats['stages']['sample_enqueue']['count']==1
        assert stats['stages']['status_write']['max_ns']==150_000_000
        assert stats['stages']['status_write']['over_100ms']==20
        assert len(stats['slow_stages'])==16
        archive.request(dict(id='finish',op='finish',session='timing_123'))
        row=json.loads((tmp_path/'media/timing_123/camera_samples.jsonl').read_text())
        assert row['sources']['rgb']['metadata']['host_perf_counter_ns']==source_ns
    finally:archive.close()


def test_blocked_sample_journal_never_blocks_sampling_and_seal_drains_original_rows(tmp_path,monkeypatch):
    import threading
    from e6_phosphobot.media_archive import ArchiveStore
    entered=threading.Event();release=threading.Event()
    original=ArchiveStore._write_sample
    def blocked(directory,line):
        entered.set();assert release.wait(3)
        original(directory,line)
    monkeypatch.setattr(ArchiveStore,'_write_sample',staticmethod(blocked))
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='start',op='start',session='queue_123'))
        offer(archive);archive.sample();assert entered.wait(1)
        started=time.monotonic()
        for index in range(2,22):offer(archive,index);archive.sample()
        assert time.monotonic()-started<.5
        assert archive.sample_index==21 and archive.store.queue.qsize()<=128
        release.set();archive.request(dict(id='finish',op='finish',session='queue_123'))
        rows=[json.loads(s) for s in (tmp_path/'media/queue_123/camera_samples.jsonl').read_text().splitlines()]
        assert [r['sample_index'] for r in rows]==list(range(21))
        assert [r['sources']['rgb']['device_frame_id'] for r in rows]==list(range(1,22))
        assert archive.status()['sample_records']==21 and archive.status()['unique_files']==42
    finally:release.set();archive.close()


def test_journal_write_failure_is_retained_and_frame_acquisition_remains_usable(tmp_path,monkeypatch):
    from e6_phosphobot.media_archive import ArchiveStore
    def fail(*args):raise OSError('injected sample journal disk failure')
    monkeypatch.setattr(ArchiveStore,'_write_sample',staticmethod(fail))
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='start',op='start',session='failure_123'))
        offer(archive);archive.sample()
        deadline=time.monotonic()+1
        while archive.store.error is None and time.monotonic()<deadline:time.sleep(.005)
        assert 'injected sample journal disk failure' in archive.store.error
        offer(archive,2)
        assert archive.latest['rgb'][0]['index']==2
        archive.request(dict(id='finish',op='finish',session='failure_123'))
        assert archive.status()['phase']=='failed' and not archive.status()['complete']
    finally:archive.close()


def test_three_second_media_backlog_drains_without_losing_rows(tmp_path,monkeypatch):
    import threading
    from e6_phosphobot.media_archive import ArchiveStore
    entered,release=threading.Event(),threading.Event()
    original=ArchiveStore._write_sample
    def blocked(directory,line):
        entered.set();assert release.wait(5)
        original(directory,line)
    monkeypatch.setattr(ArchiveStore,'_write_sample',staticmethod(blocked))
    archive=CameraArchive(tmp_path)
    try:
        archive.request(dict(id='start',op='start',session='burst_123'))
        offer(archive);archive.sample();assert entered.wait(1)
        # Sixty snapshots represent three seconds at the production 20 Hz;
        # hold the writer with an event rather than slowing the test clock.
        for index in range(2,62):offer(archive,index);archive.sample()
        status=archive.status()
        assert status['queue_peak']>128 and status['error'] is None
        assert 0<status['buffer_payload_bytes']<=status['buffer_payload_limit_bytes']
        release.set();archive.request(dict(id='finish',op='finish',session='burst_123'))
        status=archive.status()
        assert status['complete'] and status['unique_files']==122
        assert status['sample_records']==61 and status['buffer_payload_bytes']==0
        rows=[json.loads(s) for s in (tmp_path/'media/burst_123/camera_samples.jsonl').read_text().splitlines()]
        assert [r['sample_index'] for r in rows]==list(range(61))
    finally:release.set();archive.close()


@pytest.mark.parametrize('failure',['bytes','items'])
def test_media_buffer_counts_inflight_utf8_and_still_aborts_overload(tmp_path,monkeypatch,failure):
    import threading
    from e6_phosphobot.media_archive import ArchiveStore
    entered,release=threading.Event(),threading.Event()
    original=ArchiveStore._write_sample
    def blocked(directory,line):
        entered.set();assert release.wait(5)
        original(directory,line)
    monkeypatch.setattr(ArchiveStore,'_write_sample',staticmethod(blocked))
    store=ArchiveStore(tmp_path,queue_size=1,max_buffer_bytes=6 if failure=='bytes' else 100)
    try:
        store.begin('bounded_123')
        store.enqueue_sample(0,'"中"\n');assert entered.wait(1)
        assert store.status()['buffer_payload_bytes']==6
        if failure=='items':store.enqueue_sample(1,'{}\n')
        with pytest.raises(RuntimeError,match='payload buffer' if failure=='bytes' else 'queue overflow'):
            store.enqueue_sample(2,'{}\n')
        assert store.status()['buffer_payload_bytes']==(6 if failure=='bytes' else 9)
        release.set();status=store.seal('bounded_123')
        assert not status['complete'] and not status['active']
        assert status['buffer_payload_bytes']==0
        assert status['sample_records']==(1 if failure=='bytes' else 2)
    finally:release.set();store.close()
