import json
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from e6_phosphobot.pc_bridge import install_hardware
from e6_phosphobot.episode_browser import sample_page


def write_rows(path,values):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(v)+'\n' for v in values),encoding='utf-8')


def test_browser_uses_prior_received_input_and_keeps_terminal_sends_and_receipts(tmp_path):
    directory=tmp_path/'fixture_001'
    cycles=[dict(event='cycle',at_ns=t,state={'accepted':None,'actual':{'joints_rad':[1]*6}}) for t in (10,20,30)]
    inputs=[dict(event='input',at_ns=t,connection_id=c,data={'op':'intent','seq':1}) for t,c in ((9,1),(21,2))]
    events=[dict(event='motion_submitted',at_ns=31,receipt={'sent_ns':31,'accepted':False}),
            dict(event='motion_uncertain',at_ns=40,receipt={'sent_ns':31})]
    write_rows(directory/'audit/events.jsonl',inputs+cycles+events)
    write_rows(directory/'media/camera_samples.jsonl',[dict(x5_sample_ns=15,sources={'rgb':{'device_frame_id':7}}),
                                                        dict(x5_sample_ns=35,sources={'rgb':{'device_frame_id':99}})])
    write_rows(directory/'associations.jsonl',[dict(action_sent_ns=31,valid=False)])
    app=FastAPI();install_hardware(app,'ws://127.0.0.1:8776',episodes_dir=tmp_path)
    client=TestClient(app)
    first=client.get('/e6/episodes/fixture_001/samples?offset=0&limit=1').json()
    assert first['received_inputs'][0]['at_ns']==9
    assert first['media'][0]['sources']=={} # never borrow a future camera sample
    middle=client.get('/e6/episodes/fixture_001/samples?offset=1&limit=1').json()
    assert middle['received_inputs'][0]['connection_id']==1 # same seq in later connection is still future
    assert middle['events']==[] and middle['associations']==[]
    last=client.get('/e6/episodes/fixture_001/samples?offset=2&limit=1').json()
    assert last['received_inputs'][0]['connection_id']==2
    assert last['cycles'][0]['state']['accepted'] is None # actual state never becomes an action
    assert last['media'][0]['sources']['rgb']['device_frame_id']==7
    assert last['events']==events and last['associations']==[dict(action_sent_ns=31,valid=False)]
    assert last['window']==dict(begin_ns=30,end_ns=41)
    assert client.get('/e6/episodes/fixture_001/samples?offset=3').json()['cycles']==[]
    assert client.get('/e6/episodes/fixture_001/samples?offset=-1').status_code==400
    assert client.get('/e6/episodes/fixture_001/samples?limit=101').status_code==400


def test_legacy_empty_episode_does_not_invent_input_or_actions(tmp_path):
    app=FastAPI();install_hardware(app,'ws://127.0.0.1:8776',episodes_dir=tmp_path)
    result=TestClient(app).get('/e6/episodes/fixture_001/samples').json()
    assert result==dict(total=0,cycles=[],media=[],received_inputs=[],window=dict(begin_ns=0,end_ns=0),events=[],associations=[])


def native_frame(source,fid,received,available):
    return dict(source=source,id=fid,x5_written_monotonic_ns=2_000_000_000,
                metadata=dict(device_frame_id=fid,x5_receive_monotonic_ns=received,
                              metadata=dict(host_perf_counter_ns=available,capture_epoch='epoch')))


def test_browser_selects_sdk_available_frames_without_changing_originals(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    cycles=[dict(event='cycle',at_ns=n,state={}) for n in (1_000_000_000,1_010_000_000)]
    write_rows(tmp_path/'audit/events.jsonl',cycles)
    media=[native_frame('rgb',1,880_000_000,900_000_000),
           native_frame('rgb',2,970_000_000,999_000_000),
           native_frame('rgb',3,990_000_000,1_020_000_000),
           native_frame('depth',8,980_000_000,1_005_000_000)]
    write_rows(tmp_path/'media/manifest.jsonl',media)
    samples=[dict(x5_sample_ns=950_000_000,sources={'rgb':media[0]['metadata']}),
             dict(x5_sample_ns=1_025_000_000,sources={'rgb':media[2]['metadata']})]
    write_rows(tmp_path/'media/camera_samples.jsonl',samples)
    write_rows(tmp_path/'associations.jsonl',[dict(action_sent_ns=1_001_000_000,valid=False)])
    originals={p.relative_to(tmp_path):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    first=sample_page(tmp_path,0,1)['media'][0]
    assert first['selection_basis']=='sdk_available_v2'
    assert first['recorded_sample']==samples[0]
    assert first['sources']['rgb']['device_frame_id']==2 # written/sampled later, SDK already available
    assert first['sources']['rgb']['cycle_valid'] is True
    assert first['sources']['rgb']['cycle_reused'] is False
    assert 'fresh' not in first['sources']['rgb'] # do not invent acquisition metadata
    assert first['sources']['depth']['device_frame_id'] is None # SDK still in the future
    last=sample_page(tmp_path,1,1)['media'][0]
    assert last['sources']['rgb']['device_frame_id']==2 # old UVC receipt cannot admit frame 3
    assert last['sources']['rgb']['cycle_reused'] is True # stable across page boundaries
    assert last['sources']['depth']['device_frame_id']==8
    assert last['sources']['depth']['cycle_reused'] is False
    assert {p.relative_to(tmp_path):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}==originals


def test_browser_first_cycle_can_use_frame_before_first_camera_sample(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    write_rows(tmp_path/'audit/events.jsonl',[dict(event='cycle',at_ns=1_000_000_000,state={})])
    frame=native_frame('rgb',2,970_000_000,999_000_000)
    write_rows(tmp_path/'media/manifest.jsonl',[frame])
    write_rows(tmp_path/'media/camera_samples.jsonl',[dict(x5_sample_ns=1_020_000_000,sources={'rgb':frame['metadata']})])
    selected=sample_page(tmp_path,0,1)['media'][0]
    assert selected['recorded_sample'] is None
    assert selected['sources']['rgb']['device_frame_id']==2


def test_browser_missing_timestamps_stay_unknown_and_stale_stays_invalid(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    write_rows(tmp_path/'audit/events.jsonl',[dict(event='cycle',at_ns=1_000_000_000,state={})])
    write_rows(tmp_path/'media/manifest.jsonl',[
        native_frame('rgb',1,899_000_000,900_000_000),native_frame('rgb',2,999_000_000,None),
        native_frame('depth',3,None,999_000_000)])
    selected=sample_page(tmp_path,0,1)['media'][0]['sources']
    assert selected['rgb']['device_frame_id']==1 and not selected['rgb']['cycle_valid']
    assert selected['depth']['device_frame_id'] is None and not selected['depth']['cycle_valid']


def test_v2_without_manifest_keeps_legacy_sample_selection(tmp_path):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    write_rows(tmp_path/'audit/events.jsonl',[dict(event='cycle',at_ns=100,state={})])
    sample=dict(x5_sample_ns=90,sources={'rgb':{'device_frame_id':7}})
    write_rows(tmp_path/'media/camera_samples.jsonl',[sample])
    assert sample_page(tmp_path,0,1)['media']==[dict(sample,association_sample_ns=100)]


@pytest.mark.parametrize('received,available',[(101,100),(-1,100),(100,float('nan'))])
def test_browser_rejects_invalid_clock_metadata(tmp_path,received,available):
    (tmp_path/'metadata.json').write_text(json.dumps({'action_association_policy':'sdk_available_v2'}))
    write_rows(tmp_path/'media/manifest.jsonl',[native_frame('rgb',1,received,available)])
    with pytest.raises(ValueError,match='Invalid camera source'):
        sample_page(tmp_path,0,1)
