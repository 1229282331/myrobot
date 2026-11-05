"""Bounded background JSONL writer; control and GUI never wait for disk I/O."""
from datetime import datetime
import json
from queue import Queue, Full, Empty
import threading
import time


class Recorder:
    def __init__(self, path, on_error=lambda *_: None, capacity=256):
        self.path = path
        self.on_error = on_error
        self.queue = Queue(maxsize=capacity)
        self.halt = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, name="omnipicker-recorder", daemon=True)
        self.thread.start()

    def _fail(self, message):
        if self.error is None:
            self.error = message
            self.on_error(message)

    def submit(self, kind, data, metadata):
        if self.error or self.halt.is_set(): return False
        try:
            self.queue.put_nowait((kind, data, dict(metadata)))
            return True
        except Full:
            self._fail("记录队列已满，文件不完整；请停止采集")
            return False

    def _run(self):
        try:
            with self.path.open("x", encoding="utf-8", buffering=65536) as f:
                last_flush = time.monotonic()
                while not self.halt.is_set() or not self.queue.empty():
                    try:
                        kind, data, metadata = self.queue.get(timeout=.1)
                    except Empty:
                        f.flush()
                        continue
                    records = data if kind == "observation" else [data]
                    for record in records:
                        payload = dict(metadata, **record, record_type=kind,
                                       recorded_at=datetime.now().astimezone().isoformat(timespec="milliseconds"))
                        f.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n")
                    if time.monotonic() - last_flush >= .2:
                        f.flush()
                        last_flush = time.monotonic()
                if self.error:
                    f.write(json.dumps(dict(record_type="recording_error", message=self.error), ensure_ascii=False)+"\n")
        except Exception as exc:
            self._fail("日志写入失败：" + str(exc))

    def close(self):
        self.halt.set()
        self.thread.join(timeout=5)
        if self.thread.is_alive(): self._fail("日志尚未写完，文件可能不完整")
        if self.error:
            try:
                self.path.with_suffix(".incomplete.txt").write_text(self.error, encoding="utf-8")
            except OSError:
                pass  # The UI also reports the error when the disk itself fails.
