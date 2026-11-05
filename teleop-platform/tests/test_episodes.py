import io
import json
import time
import zipfile
from pathlib import Path
from types import SimpleNamespace
import pytest
from PIL import Image
from e6_phosphobot.episodes import Episodes,media_capacity
from e6_phosphobot.camera_archive import CameraArchive
from e6_phosphobot.episode_http import start_server
from e6_phosphobot.episode_transfer import TransferWorker,verify_bundle,associate,digest_file,safe_name
from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.audit import Audit


def wait(test,timeout=6):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if test():return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for state')


@pytest.fixture
def setup(tmp_path):
    camera=CameraArchive(tmp_path/'camera');camera.start()
    buffer=io.BytesIO();Image.new('RGB',(2,1)).save(buffer,format='JPEG')
    for stream,data in [('color',buffer.getvalue()),('depth',b'\0\1\2\3')]:
        camera.offer(dict(stream=stream,index=1,host_perf_counter_ns=time.perf_counter_ns(),width=2,height=1),data)
    reader=FeedbackReader('unused')
    episodes=Episodes(tmp_path/'episodes',tmp_path/'camera',reader,'shadow')
    yield episodes,camera,reader
    episodes.close();camera.close()
    assert not reader._subscriptions


def start(episodes):
    episodes.request('start',task='test',requested_ns=time.perf_counter_ns(),state={})
    wait(lambda:episodes.state()['phase']=='capturing')


def test_capacity_reaches_page_state_before_limit_without_aborting(setup):
    episodes,camera,_=setup
    camera.store.max_bytes=700
    wait(lambda:episodes.state().get('capacity') is not None)
    assert episodes.state()['capacity']['used_bytes']==0
    start(episodes)
    wait(lambda:(episodes.state().get('capacity') or {}).get('warning'))
    capacity=episodes.state()['capacity']
    assert capacity['limit_bytes']==700 and 560<=capacity['used_bytes']<700
    assert capacity['remaining_bytes']==700-capacity['used_bytes']
    assert capacity['accounting']=='native_image_payload'
    before=capacity['used_bytes']
    camera.sample();camera.sample()  # Reusing selected images must not grow usage.
    assert camera.store.status()['native_bytes']==before
    assert episodes.state()['phase']=='capturing' and not camera.store.error
    episodes.request('finish',result='success')
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    assert episodes.state()['capacity']['used_bytes']==before
    assert episodes.state()['result']=='success'


@pytest.mark.parametrize('camera',[None, [], {}, {'native_bytes':12},
    {'native_bytes':12,'native_budget_bytes':0}, {'native_bytes':-1,'native_budget_bytes':100},
    {'native_bytes':True,'native_budget_bytes':100}])
def test_unavailable_capacity_is_not_reported_as_empty_or_unlimited(camera):
    assert media_capacity(camera) is None


def test_capacity_warning_includes_threshold_and_does_not_count_compressed_disk_bytes():
    camera=dict(native_bytes=79,native_budget_bytes=100,storage_bytes=800)
    assert not media_capacity(camera)['warning']
    assert media_capacity(dict(camera,native_bytes=80))['warning']
    assert media_capacity(dict(camera,native_bytes=120))['remaining_bytes']==0


@pytest.mark.parametrize('result',['success','failure','aborted'])
def test_original_page_episode_bundle_transfer_and_ack(setup,tmp_path,result):
    episodes,camera,reader=setup
    context={'scene':' 固定局部路线 ','operator':'auto-v169'}
    episodes.request('start',task='test',requested_ns=time.perf_counter_ns(),research_context=context)
    context['scene']='changed after request'
    wait(lambda:episodes.state()['phase']=='capturing')
    reader.generation='test';reader._publish(SimpleNamespace(received_ns=time.perf_counter_ns(),raw=b'original',prefix=b'prefix'))
    episodes.put(dict(event='input',at_ns=time.perf_counter_ns(),data={'deadman':False}))
    episodes.put(dict(event='cycle',at_ns=time.perf_counter_ns(),state={'actual':'raw'}))
    wait(lambda:camera.public_status['sample_index']>0)
    episodes.request('finish',result=result)
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    with pytest.raises(ValueError):episodes.request('start',task='must wait')
    server=start_server(episodes,0)
    worker=TransferWorker('http://127.0.0.1:'+str(server.server_port),tmp_path/'pc')
    try:
        item=worker.get('/episodes')['episodes'][0]
        status=worker.one(item)
        assert status['phase']=='verified',status
        assert status['verification']['native_files']==2
        assert status['verification']['raw_frames']==1
        assert episodes.state()['phase']==('aborted' if result=='aborted' else 'complete')
        directory=tmp_path/'pc'/item['episode']
        summary=json.loads((directory/'summary.json').read_text())
        assert summary['result']==result
        assert summary['complete']==(result!='aborted')
        metadata=json.loads((directory/'metadata.json').read_text(encoding='utf-8'))
        assert metadata['dataset_kind']=='hardware_shadow'
        assert metadata['research_context']=={'scene':'固定局部路线','operator':'auto-v169'}
        assert status['verification']['association_policy']=='sdk_available_v2'
        # A lost receipt can be acknowledged again without another camera start.
        assert worker.one(item)['phase']=='verified'
    finally:server.shutdown();server.server_close()


@pytest.mark.parametrize('context',[[],{'scene':42},{'operator':'x'*81},{'scene':'x'*501},{'dataset_kind':'hardware_demonstration'}])
def test_invalid_context_rejected_before_episode_state_or_io_changes(setup,context):
    episodes,_,_=setup
    with pytest.raises(ValueError):episodes.request('start',task='test',research_context=context)
    assert episodes.state()['phase']=='idle' and episodes.directory is None
    assert not list(episodes.root.glob('*'))


def test_finish_during_start_seals_once_and_releases_subscription(setup):
    episodes,_,reader=setup
    episodes.request('start',task='immediate end',requested_ns=1)
    episodes.request('finish',result='aborted',reason='page closed')
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    assert not reader._subscriptions
    events=(episodes.directory/'audit/events.jsonl').read_text()
    assert events.count('episode_finished')==1


@pytest.mark.parametrize('stop_uncertain',[False,True])
def test_deferred_capture_preserves_finish_decision_and_full_final_state(setup,stop_uncertain):
    import threading
    from test_gateway import setup as setup_gateway
    from e6_phosphobot.state_observation import capture_audit_observation,render_observation
    episodes,_,_=setup
    # This is the recording lifecycle only; setup_gateway uses a fake device.
    episodes.stage='live'
    start(episodes)
    executor,_,_,_,_=setup_gateway()
    executor.stationary=True;executor.stop_failed=stop_uncertain
    captured=capture_audit_observation(executor,threading.RLock())
    expected=render_observation(captured)
    episodes.put(dict(event='cycle',at_ns=captured.captured_ns,state=captured))
    episodes.request('finish',result='success')
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    summary=json.loads((episodes.directory/'summary.json').read_text(encoding='utf8'))
    assert summary['result']==('aborted' if stop_uncertain else 'success')
    rows=[json.loads(line) for line in (episodes.directory/'audit/events.jsonl').read_text(encoding='utf8').splitlines()]
    assert next(row for row in rows if row['event']=='cycle')['state']==expected
    assert next(row for row in rows if row['event']=='episode_finished')['final_state']==expected


def test_camera_start_failure_retains_abort_and_closes_writer(setup):
    episodes,_,reader=setup
    original=episodes._camera_request
    def fail(op,episode):
        if op=='start':raise OSError('injected camera start failure')
        return original(op,episode)
    episodes._camera_request=fail
    episodes.request('start',task='fail',requested_ns=1)
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    assert not reader._subscriptions
    summary=json.loads((episodes.directory/'summary.json').read_text(encoding='utf-8'))
    assert summary['result']=='aborted' and 'injected' in summary['reason']


def test_initial_metadata_disk_failure_cannot_touch_previous_episode(setup,monkeypatch):
    episodes,_,reader=setup
    import e6_phosphobot.episodes as module
    original=module.atomic_json
    def fail(path,data):
        if path.name=='metadata.json':raise OSError('disk full')
        return original(path,data)
    monkeypatch.setattr(module,'atomic_json',fail)
    episodes.request('start',task='disk failure',requested_ns=1)
    wait(lambda:episodes.state()['phase'] in ('transfer_pending','aborted'))
    assert not reader._subscriptions
    assert 'disk full' in episodes.state()['error']


def test_audit_overflow_aborts_without_killing_reader(setup):
    episodes,_,reader=setup;start(episodes)
    episodes.writer.error='injected disk failure'
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    assert episodes.state()['result']=='aborted'
    reader._publish(SimpleNamespace(received_ns=5,raw=b'alive',prefix=b''))
    assert reader.latest.raw==b'alive' and not reader._subscriptions


def test_transfer_receipt_disk_failure_does_not_report_complete(setup,monkeypatch):
    episodes,_,_=setup;start(episodes)
    episodes.request('finish',result='success')
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    before=episodes.state()
    import e6_phosphobot.episodes as module
    original=module.atomic_json
    def fail(path,data):
        if path.name=='transfer.json' and data.get('phase')=='verified':
            raise OSError('injected receipt disk full')
        return original(path,data)
    monkeypatch.setattr(module,'atomic_json',fail)
    with pytest.raises(OSError,match='disk full'):
        episodes.transferred(before['episode'],before['bundle_sha256'])
    assert episodes.state()['phase']=='transfer_pending'
    assert json.loads((episodes.directory/'transfer.json').read_text())['phase']=='pending'
    monkeypatch.setattr(module,'atomic_json',original)
    # Receipt success preserves the original recording result, including a
    # separately aborted camera recording; it cannot upgrade it to success.
    expected='aborted' if before.get('result')=='aborted' else 'complete'
    assert episodes.transferred(before['episode'],before['bundle_sha256'])['phase']==expected


@pytest.mark.parametrize('blocked_io',['read','write'])
def test_slow_transfer_receipt_keeps_status_and_stop_responsive(setup,monkeypatch,blocked_io):
    import threading
    import e6_phosphobot.episodes as module
    from test_gateway import setup as executor_setup
    episodes,_,reader=setup;start(episodes)
    episodes.request('finish',result='success')
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    before=episodes.state();entered=threading.Event();resume=threading.Event()
    stop_done=threading.Event();failures=[]
    original_write=module.atomic_json;original_read=Path.read_text
    def blocked():
        entered.set()
        if not resume.wait(3):raise TimeoutError('test failed to release receipt I/O')
    def write(path,data):
        if path.name=='transfer.json' and data.get('phase')=='verified':blocked()
        return original_write(path,data)
    def read(path,*a,**kw):
        if path==episodes.directory/'transfer.json':blocked()
        return original_read(path,*a,**kw)
    monkeypatch.setattr(module,'atomic_json',write if blocked_io=='write' else original_write)
    monkeypatch.setattr(Path,'read_text',read if blocked_io=='read' else original_read)
    def acknowledge():
        try:episodes.transferred(before['episode'],before['bundle_sha256'])
        except Exception as exc:failures.append(str(exc))
    # At the device boundary this fixture deliberately requires a Stop even
    # while media is returning: journal I/O cannot decide if Stop is possible.
    executor,device,send,step,_=executor_setup()
    send(deadman=True,joint_rad_s=[0,0,0,0,0,.01]);step()
    executor.episodes=episodes
    def stop():
        try:
            executor.suspend('injected disconnect during receipt I/O')
            assert device.calls[-1][0]=='Stop'
            assert episodes.state()['phase']=='transfer_pending'
            stop_done.set()
        except Exception as exc:failures.append(str(exc))
    ack=threading.Thread(target=acknowledge);ack.start()
    assert entered.wait(1)
    stopping=threading.Thread(target=stop);stopping.start()
    try:
        # A deterministic blocked disk operation, not a real robot timing claim.
        responsive=stop_done.wait(.1)
        reader._publish(SimpleNamespace(received_ns=5,raw=b'alive',prefix=b''))
        assert reader.latest.raw==b'alive'
    finally:
        resume.set();ack.join(2);stopping.join(2)
    assert responsive, 'Disk receipt blocked state/Stop until I/O was released'
    assert not failures and not ack.is_alive() and not stopping.is_alive()
    assert episodes.state()['phase']==('aborted' if before.get('result')=='aborted' else 'complete')


def test_concurrent_duplicate_receipts_remain_idempotent(setup):
    from concurrent.futures import ThreadPoolExecutor
    episodes,_,_=setup;start(episodes)
    episodes.request('finish',result='failure')
    wait(lambda:episodes.state()['phase']=='transfer_pending')
    before=episodes.state()
    with ThreadPoolExecutor(max_workers=4) as pool:
        calls=[pool.submit(episodes.transferred,before['episode'],before['bundle_sha256']) for _ in range(4)]
        states=[call.result(timeout=2) for call in calls]
    expected='aborted' if before.get('result')=='aborted' else 'complete'
    assert all(s['phase']==expected and s['result']==before['result'] for s in states)
    saved=json.loads((episodes.directory/'transfer.json').read_text())
    assert saved['phase']=='verified' and saved['bundle_sha256']==before['bundle_sha256']


def write_rows(path,items):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(item)+'\n' for item in items),encoding='utf-8')


def test_associations_never_substitute_future_observation_or_state_as_action(tmp_path):
    obs=dict(generation='g',sequence=1,received_ns=900_000_000)
    write_rows(tmp_path/'audit/raw.jsonl',[obs,dict(generation='g',sequence=2,received_ns=1_010_000_000)])
    write_rows(tmp_path/'audit/events.jsonl',[
        dict(event='motion_submitted',at_ns=1_000_000_001,receipt={'sent_ns':1_000_000_000},pre_action_observation=obs),
        dict(event='motion_submitted',at_ns=2_000_000_001,receipt={'sent_ns':2_000_000_000},pre_action_observation=obs)])
    write_rows(tmp_path/'media/camera_samples.jsonl',[
        dict(x5_sample_ns=950_000_000,sources={'rgb':dict(device_frame_id=1,x5_receive_monotonic_ns=930_000_000)}),
        dict(x5_sample_ns=1_001_000_000,sources={'rgb':dict(device_frame_id=2,x5_receive_monotonic_ns=1_001_000_000)})])
    report=associate(tmp_path,[dict(source='rgb',id=1),dict(source='rgb',id=2)])
    assert report['invalid_action_associations']==1
    result=[json.loads(x) for x in (tmp_path/'associations.jsonl').read_text().splitlines()]
    assert result[0]['media']['rgb']['frame_id']==1 and result[0]['valid']
    assert not result[1]['valid'] and result[1]['feedback']['sequence']==1
    assert all('action' not in row for row in result)
    assert report['association_policy']=='sample_before_action_v1'


def test_v2_selects_already_available_image_even_when_sampled_later(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    obs=dict(generation='g',sequence=1,received_ns=990_000_000)
    write_rows(tmp_path/'audit/raw.jsonl',[obs])
    write_rows(tmp_path/'audit/events.jsonl',[
        dict(event='motion_submitted',receipt={'sent_ns':n},pre_action_observation=obs)
        for n in (1_000_000_000,1_001_000_000)])
    def frame(fid,received,available):
        return dict(source='rgb',id=fid,metadata=dict(device_frame_id=fid,
            x5_receive_monotonic_ns=received,
            metadata=dict(host_perf_counter_ns=available,capture_epoch='epoch')))
    media=[frame(1,900_000_000,940_000_000),frame(2,965_000_000,995_000_000),
           frame(3,990_000_000,1_010_000_000)]
    write_rows(tmp_path/'media/camera_samples.jsonl',[
        dict(x5_sample_ns=950_000_000,sources={'rgb':media[0]['metadata']}),
        dict(x5_sample_ns=1_005_000_000,sources={'rgb':media[1]['metadata']}),
        dict(x5_sample_ns=1_015_000_000,sources={'rgb':media[2]['metadata']})])
    report=associate(tmp_path,media)
    result=[json.loads(x) for x in (tmp_path/'associations.jsonl').read_text().splitlines()]
    assert report['invalid_action_associations']==0
    assert [r['media']['rgb']['frame_id'] for r in result]==[2,2]
    assert result[0]['media']['rgb']['sdk_available_ns']==995_000_000
    assert not result[0]['media']['rgb']['reused'] and result[1]['media']['rgb']['reused']
    # Retries must reproduce prior associations, never silently replace them.
    assert associate(tmp_path,media)==report
    media[1]['metadata']['x5_receive_monotonic_ns']=960_000_000
    with pytest.raises(ValueError,match='retained action associations differ'):associate(tmp_path,media)


def test_v2_cannot_make_a_stale_or_future_available_image_valid(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    write_rows(tmp_path/'audit/raw.jsonl',[dict(generation='g',sequence=1,received_ns=999_000_000)])
    write_rows(tmp_path/'audit/events.jsonl',[dict(event='can',data=dict(kind='tx',host_perf_counter_ns=1_000_000_000))])
    media=[dict(source='rgb',id=1,metadata=dict(device_frame_id=1,x5_receive_monotonic_ns=899_000_000,
                metadata=dict(host_perf_counter_ns=900_000_000))),
           dict(source='rgb',id=2,metadata=dict(device_frame_id=2,x5_receive_monotonic_ns=990_000_000,
                metadata=dict(host_perf_counter_ns=1_000_000_001)))]
    assert associate(tmp_path,media)['invalid_action_associations']==1
    row=json.loads((tmp_path/'associations.jsonl').read_text())
    assert row['media']['rgb']['frame_id']==1 and row['media']['rgb']['age_ms']==101


@pytest.mark.parametrize('received,available',[(101,100),(-1,100),(100,float('nan'))])
def test_v2_rejects_bad_source_clock_metadata(tmp_path,received,available):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    media=[dict(source='rgb',id=1,metadata=dict(device_frame_id=1,x5_receive_monotonic_ns=received,
                metadata=dict(host_perf_counter_ns=available)))]
    with pytest.raises(ValueError,match='Invalid camera source'):associate(tmp_path,media)


def test_unknown_association_policy_is_not_reinterpreted_as_legacy(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'future_v9'}))
    with pytest.raises(ValueError,match='Unknown action association'):associate(tmp_path,[])


def test_recording_status_uses_session_ipc_but_original_media_stays_on_disk(tmp_path):
    disk=tmp_path/'disk';status=tmp_path/'ipc/recording.status.json'
    camera=CameraArchive(disk,status_path=status);camera.start()
    # A legacy disk status is deliberately wrong and must not be consulted.
    (disk/'recording.status.json').write_text(json.dumps({'error':'stale old file'}))
    reader=FeedbackReader('unused')
    episodes=Episodes(tmp_path/'episodes',disk,reader,'shadow',camera_status_path=status)
    camera.offer(dict(stream='color',index=1,host_perf_counter_ns=time.perf_counter_ns()),b'original')
    try:
        start(episodes)
        wait(lambda:camera.public_status['sample_index']>0)
        episodes.request('finish',result='success')
        wait(lambda:episodes.state()['phase']=='transfer_pending')
        summary=json.loads((episodes.directory/'summary.json').read_text())
        assert summary['result']=='success' and summary['media']['unique_files']==1
        assert (disk/'media'/episodes.directory.name/'manifest.jsonl').is_file()
        assert not (status.parent/'media').exists()
        assert json.loads((disk/'recording.status.json').read_text())['error']=='stale old file'
        assert not reader._subscriptions
    finally:episodes.close();camera.close()


@pytest.mark.parametrize('name',['../escape','/absolute','C:/escape'])
def test_bundle_rejects_path_escape(tmp_path,name):
    bundle=tmp_path/'bad.zip'
    with zipfile.ZipFile(bundle,'w') as z:z.writestr(name,b'bad')
    with pytest.raises(ValueError,match='Unsafe'):verify_bundle(bundle,tmp_path/'pc',digest_file(bundle))


def test_windows_name_is_rejected_before_extraction():
    with pytest.raises(ValueError):safe_name('nested\\escape')
