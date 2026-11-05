import base64
import json
import os
import pickle
import subprocess
import sys
import time
from types import SimpleNamespace

from e6_phosphobot.audit import Audit
from e6_phosphobot.feedback import FeedbackReader


def wait_for(condition, timeout=5):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if condition():return
        time.sleep(.01)
    raise AssertionError('Audit completion timed out')


def test_independent_writer_preserves_raw_and_multiline_events(tmp_path):
    reader=FeedbackReader('unused');reader.generation='generation-with-prefix'
    audit=Audit(tmp_path/'audit',reader)
    expected=[]
    for i in range(500):
        reader._publish(SimpleNamespace(raw=bytes(range(256))*5,prefix=bytes([i%256]),received_ns=i))
        row=dict(event='cycle',at_ns=i,state={'text':'中文\nTraceback\n  original line','value':i/3,'valid':False,'raw':None})
        expected.append(row);audit.put(row)
    audit.close()
    assert audit.error is None and not reader._subscriptions
    assert audit.process.pid!=os.getpid() and audit.process.poll()==0
    assert audit.writer_receipt_received
    assert audit.counts==dict(raw=500,events=500)
    assert [json.loads(x) for x in (audit.path/'events.jsonl').read_text(encoding='utf-8').splitlines()]==expected
    raw=[json.loads(x) for x in (audit.path/'raw.jsonl').read_text().splitlines()]
    assert [r['sequence'] for r in raw]==list(range(1,501))
    for i,row in enumerate(raw):
        assert row['received_ns']==i and row['generation']==reader.generation
        assert base64.b64decode(row['raw_b64'])==bytes(range(256))*5
        assert base64.b64decode(row['prefix_b64'])==bytes([i%256])


def test_writer_crash_is_reported_and_receiver_remains_available(tmp_path):
    reader=FeedbackReader('unused');audit=Audit(tmp_path/'audit',reader)
    wait_for(lambda:audit.process is not None)
    audit.process.terminate()
    wait_for(lambda:audit.error is not None)
    start=time.monotonic()
    for i in range(3000):reader._publish(SimpleNamespace(raw=b'x',prefix=b'',received_ns=i))
    assert time.monotonic()-start<1
    audit.close()
    assert not reader._subscriptions and not audit.thread.is_alive()
    assert json.loads((audit.path/'summary.json').read_text())['error']


def test_raw_overflow_is_retained_as_failure(tmp_path):
    reader=FeedbackReader('unused');audit=Audit(tmp_path/'audit',reader)
    audit.raw.dropped=1
    wait_for(lambda:audit.error is not None)
    audit.close()
    assert 'audit_raw_overflow' in audit.error
    assert audit.process.poll() is not None and not reader._subscriptions


def test_writer_eof_reports_saved_partial_counts(tmp_path):
    child=subprocess.Popen([sys.executable,'-B','-m','e6_phosphobot.audit_writer',str(tmp_path)],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    rows=[('raw',('generation',i,i,b'',b'original')) for i in range(37)]
    rows+=[('events',dict(event='preserved',at_ns=38))]
    # Deliberately omit the final sentinel: this must remain a failed run.
    stdout,stderr=child.communicate(b''.join(pickle.dumps(row,protocol=5) for row in rows),timeout=10)
    assert child.returncode==1 and not stderr
    result=json.loads(stdout)
    assert result['error'].startswith('EOFError')
    assert result['counts']==dict(raw=37,events=1)
    assert len((tmp_path/'raw.jsonl').read_text().splitlines())==37
    assert len((tmp_path/'events.jsonl').read_text().splitlines())==1


def test_overflow_retains_partial_writer_receipt(tmp_path):
    reader=FeedbackReader('unused');audit=Audit(tmp_path/'audit',reader)
    try:
        for i in range(100):
            reader._publish(SimpleNamespace(raw=b'x'*1440,prefix=b'',received_ns=i))
        # Wait until buffered raw output has reached disk before provoking a
        # real failure path in the parent.
        raw=audit.path/'raw.jsonl'
        wait_for(lambda:raw.exists() and raw.stat().st_size>0)
        audit.raw.dropped=1
        wait_for(lambda:audit.error is not None)
    finally:audit.close()
    saved=len(raw.read_text().splitlines())
    assert saved>0 and audit.error=='audit_raw_overflow'
    assert audit.writer_receipt_received and audit.counts['raw']==saved
