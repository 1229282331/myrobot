"""Exercise storage stalls, original-byte integrity and bounded admission."""
import hashlib
import json
from pathlib import Path
import threading

import pytest
from e6_phosphobot.media_archive import ArchiveStore


def frame(index,data):
    return dict(sources={'rgb':dict(device_frame_id=index,jpeg=data,
        original_sha256=hashlib.sha256(data).hexdigest(),original_bytes=len(data)),
        'depth':{}},stats={})


def test_compression_continues_during_blocked_disk_with_original_bytes_retained(tmp_path,monkeypatch):
    entered,release=threading.Event(),threading.Event()
    original=Path.write_bytes
    def blocked(path,data):
        if path.suffix=='.bin' and not entered.is_set():
            entered.set();assert release.wait(5)
        return original(path,data)
    monkeypatch.setattr(Path,'write_bytes',blocked)
    store=ArchiveStore(tmp_path,max_buffer_bytes=256*1024)
    payload=bytes(range(256))*256
    try:
        store.begin('pipeline_123')
        store.capture(frame(0,payload));assert entered.wait(1)
        for index in range(1,41):
            store.capture(frame(index,payload))
            store.enqueue_sample(index,json.dumps(dict(index=index))+'\n')
            # Wait only for encoding; the disk remains deliberately blocked.
            store.queue.join()
        status=store.status()
        assert status['error'] is None and status['native_bytes']>10*status['buffer_payload_limit_bytes']
        assert status['buffer_payload_bytes']<status['buffer_payload_limit_bytes']
        assert status['storage_queue_size']==80 and status['unique_files']==0
        release.set();status=store.seal('pipeline_123')
        assert status['complete'] and status['unique_files']==41 and status['sample_records']==40
        assert status['buffer_payload_bytes']==0 and status['queue_size']==0
        for index in range(41):
            entry,data=store.fetch('pipeline_123','rgb',index)
            assert data==payload and entry['bytes']==len(payload)
        assert [json.loads(s)['index'] for s in (tmp_path/'pipeline_123/camera_samples.jsonl').read_text().splitlines()]==list(range(1,41))
    finally:release.set();store.close()
    assert not store.worker.is_alive() and not store.compressor.is_alive()


def test_compressed_stage_does_not_double_total_item_allowance(tmp_path,monkeypatch):
    entered,release=threading.Event(),threading.Event()
    def blocked(directory,line):entered.set();assert release.wait(3)
    monkeypatch.setattr(ArchiveStore,'_write_sample',staticmethod(blocked))
    store=ArchiveStore(tmp_path,queue_size=2)
    try:
        store.begin('items_123');store.enqueue_sample(0,'{}\n');assert entered.wait(1)
        store.enqueue_sample(1,'{}\n');store.queue.join()
        store.enqueue_sample(2,'{}\n');store.queue.join()
        with pytest.raises(RuntimeError,match='queue overflow'):store.enqueue_sample(3,'{}\n')
        assert store.pending_items==3
        with pytest.raises(ValueError,match='sealed'):store.release('items_123')
        release.set();status=store.seal('items_123')
        assert status['sample_records']==3 and status['buffer_payload_bytes']==0
        assert not status['complete']
    finally:release.set();store.close()


def test_checksum_failure_before_encoding_is_not_written_or_silently_retried(tmp_path):
    store=ArchiveStore(tmp_path)
    try:
        store.begin('checksum_123')
        bad=frame(0,b'original');bad['sources']['rgb']['jpeg']=b'changed!'
        store.capture(bad);store.queue.join()
        status=store.seal('checksum_123')
        assert 'changed before archive write' in status['error']
        assert status['unique_files']==0 and status['buffer_payload_bytes']==0
        assert not list((tmp_path/'checksum_123').glob('*.bin'))
    finally:store.close()
