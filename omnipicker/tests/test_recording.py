import csv
import json
from pathlib import Path
from threading import Event
from studio.recording import Recorder
from tools.export_observations import export


def test_all_samples_flushed_and_reconnect_does_not_inherit_target(tmp_path):
    path=tmp_path/'run.jsonl';errors=[]
    recorder=Recorder(path,errors.append)
    meta=dict(run_id='run',connection_id='one',simulated=False,firmware='3.3.5',serial='TEST')
    # A delayed receive batch arrives after its action log; join by sample time.
    recorder.submit('log',dict(monotonic=1.,action='set_opening',target=35.),meta)
    recorder.submit('observation',[dict(monotonic=t,host_sequence=i,position_fraction=.15,
        velocity_fraction_s=0.,column1_unverified=.01,raw='0.01,0.00,0.15',source='usb_cdc',stream_accepted=True)
        for i,t in enumerate([.9,1.,1.1])],meta)
    recorder.submit('observation',[dict(monotonic=2.,position_fraction=.15)],dict(meta,connection_id='two'))
    recorder.close()
    assert not errors and not recorder.thread.is_alive()
    records=[json.loads(line) for line in path.read_text().splitlines()]
    assert len(records)==5
    output=tmp_path/'out.csv'
    assert export(path,output)==4
    with output.open(encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
    assert [r['last_confirmed_target_percent'] for r in rows]==['','35.0','35.0','']
    assert rows[0]['raw']=='0.01,0.00,0.15'


def test_recorder_does_not_overwrite_and_reports_failure(tmp_path):
    path=tmp_path/'existing.jsonl';path.write_text('original')
    error=Event()
    recorder=Recorder(path,lambda _:error.set())
    assert error.wait(1)
    recorder.close()
    assert recorder.error and path.read_text()=='original'
    assert not recorder.submit('log',{}, {})


def test_export_rejects_partial_json_and_refuses_overwrite(tmp_path):
    import pytest
    path=tmp_path/'bad.jsonl';path.write_text('{"partial":')
    output=tmp_path/'out.csv'
    with pytest.raises(ValueError): export(path,output)
    assert not output.exists()
    path.write_text('')
    output.write_text('original')
    with pytest.raises(FileExistsError): export(path,output)
    assert output.read_text()=='original'


def test_bounded_queue_overflow_is_reported_and_export_rejects_incomplete(tmp_path, monkeypatch):
    import pytest
    opened, release = Event(), Event()
    original_open = Path.open
    path = tmp_path/'slow.jsonl'
    def slow_open(self, *args, **kwargs):
        if self == path and args and args[0] == 'x':
            opened.set()
            assert release.wait(2)
        return original_open(self, *args, **kwargs)
    monkeypatch.setattr(Path,'open',slow_open)
    recorder=Recorder(path,capacity=1)
    assert opened.wait(1)
    assert recorder.submit('log',{'message':'first'}, {})
    assert not recorder.submit('log',{'message':'overflow'}, {})
    release.set();recorder.close()
    assert recorder.error and path.with_suffix('.incomplete.txt').exists()
    with pytest.raises(ValueError): export(path,tmp_path/'should-not-exist.csv')
