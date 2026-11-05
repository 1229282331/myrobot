"""A slow evidence sink must not block live feedback or silently lose records."""
import json
import os
import signal
import time
from types import SimpleNamespace

import pytest

from e6_phosphobot.audit import Audit
from e6_phosphobot.feedback import FeedbackReader
from test_audit_process import wait_for


@pytest.mark.skipif(os.name != 'posix', reason='Private writer pause uses POSIX SIGSTOP')
def test_bounded_raw_backlog_survives_writer_pause(tmp_path):
    reader=FeedbackReader('unused');audit=Audit(tmp_path/'audit',reader)
    stopped=False
    try:
        wait_for(lambda:audit.process is not None)
        reader._publish(SimpleNamespace(raw=b'x'*1440,prefix=b'',received_ns=0))
        audit.put(dict(event='first',at_ns=0))
        time.sleep(.5)
        os.kill(audit.process.pid,signal.SIGSTOP);stopped=True
        start=time.monotonic()
        for i in range(1,4001):
            reader._publish(SimpleNamespace(raw=i.to_bytes(8,'little')+b'x'*1432,prefix=b'',received_ns=i))
            if i%6==0:audit.put(dict(event='queued',at_ns=i))
        # Receiver and latest feedback stay independent of the paused sink.
        assert time.monotonic()-start<1
        assert reader.latest.received_ns==4000
        assert audit.raw.dropped==0
        os.kill(audit.process.pid,signal.SIGCONT);stopped=False
        wait_for(lambda:audit.raw.queue.empty() and audit.events.empty(),10)
    finally:
        if stopped:os.kill(audit.process.pid,signal.SIGCONT)
        audit.close()
    assert audit.error is None and audit.writer_receipt_received
    assert audit.counts==dict(raw=4001,events=667)
    rows=[json.loads(line) for line in (audit.path/'raw.jsonl').read_text().splitlines()]
    assert [row['sequence'] for row in rows]==list(range(1,4002))
