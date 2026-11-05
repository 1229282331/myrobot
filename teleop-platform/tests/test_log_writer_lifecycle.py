import logging
import os
import threading

import pytest

from e6_phosphobot.operator_logs import AsyncLog, _Utf8WatchedFile


def flushed(sink):
    # Bounded wait; queue completion means data has been flushed or an IO
    # failure was recorded, rather than merely copied into a Python buffer.
    done=threading.Event()
    worker=threading.Thread(target=lambda:(sink.queue.join(),done.set()),daemon=True)
    worker.start()
    assert done.wait(3)


def test_stream_reused_and_sparse_records_visible_before_close(tmp_path,monkeypatch):
    opens=[]
    original=_Utf8WatchedFile._open
    def opened(self):
        opens.append(self.baseFilename)
        return original(self)
    monkeypatch.setattr(_Utf8WatchedFile,'_open',opened)
    sink=AsyncLog(tmp_path/'control.log')
    try:
        for i in range(8):
            sink.native(f'中文 {i}\n',0)
            flushed(sink)
            assert sink.path.read_bytes().endswith(f'中文 {i}\n'.encode())
        assert len(opens)==1
    finally:sink.close()
    assert not sink.thread.is_alive() and not sink.write_errors


@pytest.mark.skipif(os.name=='nt',reason='Unix open-file rename semantics')
def test_rename_and_copytruncate_follow_visible_file(tmp_path):
    sink=AsyncLog(tmp_path/'control.log')
    try:
        sink.native('old\n',4);flushed(sink)
        old=tmp_path/'retired.log';sink.path.rename(old)
        sink.native('new\n',4);flushed(sink)
        assert old.read_bytes()==b'old\n'
        assert sink.path.read_bytes()==b'new\n'
        sink.path.write_bytes(b'')
        sink.native('after truncate\n',15);flushed(sink)
        assert sink.path.read_bytes()==b'after truncate\n'
    finally:sink.close()
    assert not sink.thread.is_alive() and not sink.write_errors


def test_shutdown_cannot_wait_for_blocked_file_close(tmp_path):
    entered=threading.Event();release=threading.Event()
    sink=AsyncLog(tmp_path/'control.log')
    original=sink.output.close
    def blocked():
        entered.set();release.wait(5);original()
    sink.output.close=blocked
    try:
        sink.emit(logging.LogRecord('test',logging.INFO,'',0,'last',(),None))
        sink.close()
        assert entered.is_set() and sink.thread.is_alive()
        assert sink.path.read_bytes().endswith(b'last\n')
    finally:
        release.set();sink.thread.join(3);sink.output.close=original
    assert not sink.thread.is_alive()


def test_flush_failure_retained_and_writer_can_exit(tmp_path,monkeypatch):
    sink=AsyncLog(tmp_path/'control.log')
    def fail():raise OSError('injected flush failure')
    monkeypatch.setattr(sink.output,'flush',fail)
    sink.native('original\n',9);sink.close()
    assert not sink.thread.is_alive()
    assert sink.write_errors and sink.error=='injected flush failure'


def test_inner_stream_is_not_registered_for_synchronous_logging_shutdown(tmp_path):
    sink=AsyncLog(tmp_path/'control.log')
    try:
        assert not isinstance(sink.output,logging.Handler)
        assert not any(ref() is sink.output for ref in logging._handlerList)
    finally:sink.close()
