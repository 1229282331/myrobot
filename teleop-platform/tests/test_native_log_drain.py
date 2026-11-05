import os
import threading

from e6_phosphobot.operator_logs import AsyncLog, NativeLogDrain


def test_native_utf8_fragments_and_unterminated_tail_preserved(tmp_path):
    read_fd,write_fd=os.pipe()
    sink=AsyncLog(tmp_path/'native.log')
    drain=NativeLogDrain(os.fdopen(read_fd,'rb'),sink)
    text='[WARN] 中文\n  original native traceback\nlast line'
    data=text.encode()
    try:
        for byte in data:os.write(write_fd,bytes([byte]))
    finally:os.close(write_fd)
    status=drain.close();sink.close()
    assert status['eof'] and not status['error'] and not status['drain_alive']
    assert status['bytes']==len(data) and status['dropped_native_bytes']==0
    assert sink.path.read_text(encoding='utf8')==text


def test_blocked_disk_cannot_block_native_pipe_drain(tmp_path):
    blocked=threading.Event();release=threading.Event();sent=threading.Event()
    class Slow(AsyncLog):
        def _write_native(self,text):blocked.set();release.wait(5)
    sink=Slow(tmp_path/'native.log',capacity=2)
    read_fd,write_fd=os.pipe();drain=NativeLogDrain(os.fdopen(read_fd,'rb'),sink)
    data=b'x'*4096
    def writer():
        try:
            for _ in range(512):os.write(write_fd,data)
        finally:os.close(write_fd);sent.set()
    worker=threading.Thread(target=writer);worker.start()
    try:
        assert blocked.wait(1)
        assert sent.wait(2), 'Native producer waited for disk IO'
        status=drain.close()
        assert status['eof'] and status['bytes']==512*len(data)
        assert sink.queue.qsize()<=2 and status['dropped_native_bytes']>0
    finally:
        release.set();worker.join(3);drain.close();sink.close()
    assert not drain.thread.is_alive() and not sink.thread.is_alive()


def test_native_disk_error_is_reported_without_killing_drain(tmp_path):
    path=tmp_path/'file';path.write_text('occupied')
    sink=AsyncLog(path/'native.log');read_fd,write_fd=os.pipe()
    drain=NativeLogDrain(os.fdopen(read_fd,'rb'),sink)
    os.write(write_fd,b'original failure\n');os.close(write_fd)
    status=drain.close();sink.close()
    assert status['eof'] and status['error'] is None
    assert sink.write_errors>0 and sink.error
