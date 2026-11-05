import asyncio
import base64
from io import BytesIO
import json
from pathlib import Path
import struct
import sys
import time
import zlib
from unittest.mock import patch
import pytest
from PIL import Image
from fastapi.testclient import TestClient
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.protocol import FeedbackParser,decode,assemble,READ_QUERIES,encode_wire,decode_wire
from e6_teleop.observation_clock import TransportClock
from e6_teleop.observation_recording import ObservationRecorder
from e6_teleop.observation_service import make_app
from e6_teleop.exporter import load_episode
from e6_teleop.recording import read_raw_episode


def packet():
    b=bytearray(1440);struct.pack_into('<H',b,0,1440);struct.pack_into('<Q',b,48,0x123456789ABCDEF)
    struct.pack_into('<6d',b,432,-66,-60,105,-90,90,9)
    return bytes(b)


def frame(seq=0,instance='one',now=None):
    now=now or time.perf_counter_ns()
    im=Image.new('RGB',(4,2),(60,100,140));buf=BytesIO();im.save(buf,'JPEG')
    rgb=dict(source='fixture',device_frame_id=seq//2,x5_receive_monotonic_ns=now-20_000_000,width=4,height=2,jpeg=base64.b64encode(buf.getvalue()).decode())
    depth=dict(rgb,uint16_raw=base64.b64encode(struct.pack('<8H',*range(8))).decode(),depth_scale_mm=1,data_valid=True,valid_fraction=.875)
    robot=dict(source='fixture',x5_receive_monotonic_ns=now-1_000_000,raw_base64=base64.b64encode(packet()).decode(),decoded=decode(packet()))
    return assemble(dict(robot=robot,rgb=rgb,depth=depth),{},instance,seq,now)


def test_feedback_stream_split_coalesced_and_truncated():
    p=FeedbackParser();b=packet()
    assert not p.feed(b[:1]);assert not p.feed(b[1:137])
    parsed=p.feed(b[137:]+b+b[:20]);assert len(parsed)==2
    assert parsed[0][1]['joints_rad'][0]==pytest.approx(-66*3.141592653589793/180)
    with pytest.raises(ValueError,match='Truncated'):p.eof()
    assert len(p.feed(b[20:]))==1;p.eof()


def test_recorded_hardware_feedback_split_and_corruption():
    path=Path(__file__).resolve().parents[2]/'docs/evidence/teleop_observation_20261004/e6_feedback_1440.bin'
    raw=path.read_bytes();p=FeedbackParser()
    assert not p.feed(raw[:13])
    rows=p.feed(raw[13:]+raw)
    assert len(rows)==2 and rows[0][1]['robot_mode']==4
    assert rows[0][1]['q_actual_deg'][0]==pytest.approx(-66.4938,abs=.01)
    p.eof()
    for offset in (432,624,1168,1248):
        broken=bytearray(raw);struct.pack_into('<d',broken,offset,float('inf'))
        with pytest.raises(ValueError,match='finite'):decode(bytes(broken))


@pytest.mark.parametrize('kind',['length','marker','nan'])
def test_invalid_feedback_rejected(kind):
    b=bytearray(packet())
    if kind=='length':struct.pack_into('<H',b,0,1439)
    elif kind=='marker':struct.pack_into('<Q',b,48,3)
    else:struct.pack_into('<d',b,432,float('nan'))
    with pytest.raises(ValueError):FeedbackParser().feed(b)


def test_staleness_optional_depth_and_association():
    f=frame(now=1_000_000_000);s=f['sources']
    latest={n:dict(v) for n,v in s.items() if n!='gripper'}
    latest['depth']['data_valid']=False
    a=assemble(latest,{},'one',1,1_000_000_000)
    assert a['observation']['valid'] and not a['sources']['depth']['valid']
    latest['rgb']['x5_receive_monotonic_ns']=850_000_000
    a=assemble(latest,{},'one',2,1_000_000_000)
    assert a['sources']['rgb']['valid'] and not a['associations']['rgb_robot']['valid']
    assert not assemble(latest,{},'one',3,1_600_000_000)['observation']['valid']
    assert a['input'] is None and a['effective_action'] is None and a['sources']['gripper']['opening'] is None
    assert READ_QUERIES==('RobotMode()','GetAngle()','GetPose()','GetErrorID()')


def test_raw_observation_labels_images_depth_and_repetition(tmp_path):
    r=ObservationRecorder(tmp_path);name=r.start('observation',{})
    r.sample(frame(0));r.sample(frame(1));r.finish('observed')
    rows=[json.loads(s) for s in (tmp_path/name/'samples.jsonl').read_text(encoding='utf-8').splitlines()]
    assert rows[1]['sources']['rgb']['repeated']
    assert all(x['input'] is None and x['effective_action'] is None for x in rows)
    with Image.open(tmp_path/name/rows[0]['sources']['depth']['file']) as im:depth=im.tobytes()
    assert struct.unpack('<8H',depth)==tuple(range(8))
    assert json.loads((tmp_path/name/'metadata.json').read_text())['dataset_kind']=='hardware_observation'
    with pytest.raises(ValueError,match='simulation'):load_episode(tmp_path/name)


@pytest.mark.parametrize('fault',['restart','gap','action','stale'])
def test_recording_rejects_contamination_or_discontinuity(tmp_path,fault):
    r=ObservationRecorder(tmp_path);r.start('observation',{});r.sample(frame(0))
    f=frame(1)
    if fault=='restart':f['instance']='two'
    elif fault=='gap':f['observation']['frame_id']=3
    elif fault=='action':f['effective_action']={}
    else:f['observation']['valid']=False
    with pytest.raises(ValueError):r.sample(f)
    assert r.incomplete
    r.finish('aborted')


class Connection:
    def __init__(self,mode):self.mode=mode
    async def __aenter__(self):return self
    async def __aexit__(self,*_):pass
    def __aiter__(self):return self.messages()
    async def send(self,message):self.probe=json.loads(message)
    async def messages(self):
        if hasattr(self,'probe'):
            for _ in range(5):yield json.dumps(dict(clock_probe=self.probe['nonce'],x5_monotonic_ns=time.perf_counter_ns()))
        for seq in range(100):
            await asyncio.sleep(.05)
            f=frame(seq)
            if seq>5:
                if self.mode=='disconnect':raise ConnectionError('injected SSH interruption')
                if self.mode=='robot':f['observation']['valid']=False;f['sources']['robot']['valid']=False
                if self.mode=='rgb':f['observation']['valid']=False;f['sources']['rgb']['valid']=False
                if self.mode=='restart':f['instance']='two'
            yield json.dumps(f)


@pytest.mark.parametrize('fault',['robot','rgb','disconnect','restart','disk'])
def test_service_fault_aborts_preserves_and_requires_explicit_start(tmp_path,fault):
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=lambda *a,**kw:Connection(fault)):
        with TestClient(make_app(tmp_path,render=False)) as c:
            for _ in range(40):
                if c.get('/api/state').json()['gateway_ready']:break
                time.sleep(.01)
            if fault=='disk':
                with patch.object(ObservationRecorder,'sample',side_effect=OSError('injected disk failure')):
                    assert c.post('/api/record/start',json={'task':'fault'}).status_code==200
                    time.sleep(.15)
            else:
                assert c.post('/api/record/start',json={'task':'fault'}).status_code==200
                time.sleep(.4)
            # Wait for the injected source error, rather than a fixed sleep
            # competing with Windows test scheduling. This is not the device
            # stale threshold; the production age checks remain unchanged.
            deadline=time.monotonic()+1
            while True:
                state=c.get('/api/state').json()
                if not state['recording']['active'] or time.monotonic()>=deadline:break
                time.sleep(.01)
            assert not state['recording']['active']
            episodes=c.get('/api/episodes').json();assert len(episodes)==1
            assert episodes[0]['result']=='aborted' and not episodes[0]['complete']
            assert c.post('/api/export/'+episodes[0]['id']).status_code==400
            with c.websocket_connect('/ws/control') as ws:assert not ws.receive_json()['ok']


def test_observation_endpoint_rejects_writes_and_remote_origin(tmp_path):
    with TestClient(make_app(tmp_path,gateway='ws://127.0.0.1:1',render=False)) as c:
        assert c.post('/api/record/start',json={'task':'x'}).status_code==409
        for operation in ('EnableRobot()','DisableRobot()','Stop()','ServoJ()','MovJ()','ClearError()','Load()','CAN TX'):
            assert c.post('/api/control',json={'operation':operation}).status_code==403
        assert c.get('/api/state',headers={'Origin':'https://external.example'}).status_code==403


def test_lossless_wire_references_and_corruption():
    f=frame();sent={};received={}
    first=encode_wire(f,sent);decoded=decode_wire(first,received)
    assert decoded==f
    second=encode_wire(f,sent)
    assert len(second)<len(first)
    assert decode_wire(second,received)==f
    with pytest.raises(ValueError,match='reference'):decode_wire(second,{})
    damaged=bytearray(first);damaged[-2]^=255
    with pytest.raises(Exception):decode_wire(bytes(damaged),{})
    import zstandard
    for raw in (b'\x00',struct.pack('<I',65537)+b'x',b'x'*(8*1024*1024+1)):
        with pytest.raises(ValueError):decode_wire(zstandard.ZstdCompressor().compress(raw))


def test_control_header_over_64k_keeps_action_and_can_evidence():
    f=frame()
    f.update(schema_version=3,backend='hardware-control',control={
        'command_events':[dict(command='ServoJ',raw_reply='0,{};',detail='x'*2800) for _ in range(20)],
        'can_events':[dict(raw_hex='000026000d190000',detail='y'*1000) for _ in range(20)],
        'tick':dict(effective_action=dict(command_id=19385,gripper=.15))})
    import zstandard
    encoded=encode_wire(f)
    unpacked=zstandard.ZstdDecompressor().decompress(encoded)
    assert struct.unpack_from('<I',unpacked)[0]>65536
    assert decode_wire(encoded)==f


def test_header_ceiling_is_shared_and_oversized_header_fails_closed():
    import zstandard
    from e6_observer.protocol import MAX_HEADER_BYTES
    f=frame();f['diagnostic']='x'*MAX_HEADER_BYTES
    with pytest.raises(ValueError,match='header'):encode_wire(f)
    raw=json.dumps(f).encode()
    with pytest.raises(ValueError,match='header'):
        decode_wire(zstandard.ZstdCompressor().compress(struct.pack('<I',len(raw))+raw))


def test_cross_host_age_bound_rejects_transport_backlog():
    c=TransportClock()
    # Unrelated clock epochs. Only elapsed intervals are compared.
    assert c.update(9_000_000_000,9_020_000_000,400_000_000_000)
    f=frame(now=400_050_000_000)
    c.apply(f,9_080_000_000)
    assert f['observation']['valid']
    assert f['transport']['age_upper_ms']==pytest.approx(30.08)
    c.apply(f,9_700_000_000)
    assert not f['observation']['valid']
    assert not c.update(1,300_000_001,100)
    assert c.age_upper_ms(400_050_000_000,40_000_000_000) is None


def test_corrupted_finish_summary_still_browsable(tmp_path):
    r=ObservationRecorder(tmp_path);name=r.start('disk failure evidence',{})
    r.sample(frame());r.finish('observed')
    (tmp_path/name/'summary.json').write_text('{"result":',encoding='utf-8')
    summary,rows=read_raw_episode(tmp_path/name)
    assert not summary['complete'] and summary['result']=='aborted' and len(rows)==1


def test_clock_anchor_refresh_before_expiry():
    c=TransportClock();assert c.update(0,20_000_000,100_000_000_000)
    for second in (5,10,15):assert not c.update(second*1_000_000_000,second*1_000_000_000+80_000_000,100_000_000_000+second*1_000_000_000)
    assert c.update(25_000_000_000,25_080_000_000,125_000_000_000)
    assert c.age_upper_ms(126_000_000_000,26_100_000_000) is not None


def test_observation_browse_uses_bounded_pages(tmp_path):
    directory=tmp_path/'hardware_observation'/'raw'
    r=ObservationRecorder(directory);name=r.start('paged observation',{})
    for i in range(205):r.sample(frame(i))
    r.finish('observed')
    with TestClient(make_app(tmp_path,gateway='ws://127.0.0.1:1',render=False)) as c:
        assert c.get('/api/episodes').json()[0]['samples']==205
        response=c.get('/api/episodes/'+name+'?offset=100&limit=100')
        rows=response.json()['rows']
        assert len(rows)==100 and rows[0]['sample_index']==100
        assert c.get('/api/episodes/'+name+'?limit=201').status_code==400


def test_clock_intersects_probes_without_a_bad_refresh_jump():
    c=TransportClock();assert c.update(0,20_000_000,100_010_000_000)
    # A high-RTT refresh cannot discard a still-valid narrow interval.
    assert c.update(21_000_000_000,21_180_000_000,121_100_000_000)
    bounds=c.bounds_ms(121_100_000_000,21_200_000_000)
    assert bounds['age_upper_ms']==pytest.approx(131.2)
    assert bounds['age_lower_ms']==pytest.approx(68.8)
    assert bounds['probes_used']==2
    f=frame(now=121_100_000_000);c.apply(f,21_800_000_000)
    assert not f['observation']['valid']


def test_clock_inconsistent_epochs_and_expiry_do_not_label_fresh():
    c=TransportClock();c.update(0,20_000_000,100_000_000_000)
    c.update(1_000_000_000,1_020_000_000,102_000_000_000)
    assert c.bounds_ms(102_010_000_000,1_050_000_000) is None
    assert c.bounds_ms(102_010_000_000,32_000_000_000) is None


def test_x5_dashboard_sends_only_fixed_read_queries():
    import importlib
    import types
    # runtime is Linux-only; exercise its dashboard loop with no native I/O.
    with patch.dict(sys.modules,{'fcntl':types.ModuleType('fcntl')}):
        runtime=importlib.import_module('e6_observer.runtime')
    hub=runtime.Hub('fixture','unused','test');sent=[]
    class Socket:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def settimeout(self,value):pass
        def sendall(self,value):
            sent.append(value.decode());self.reply=('0,{},'+value.decode()+';').encode()
            if len(sent)==4:hub.stop.set()
        def recv(self,count):return self.reply
    with patch.object(runtime.socket,'create_connection',return_value=Socket()):hub.dashboard()
    assert tuple(sent)==READ_QUERIES
    assert hub.stats['can_tx_count']==0 and 'dashboard_error' not in hub.stats


def test_session_wifi_setting_restores_on_exception():
    from e6_observer.wifi_latency import WifiLatency
    import types
    mode=WifiLatency(True);calls=[]
    def command(*args):
        calls.append(args);return types.SimpleNamespace(stdout='Power save: on')
    with patch.object(mode,'command',side_effect=command):
        with pytest.raises(RuntimeError):
            with mode:
                assert mode.report['changed']
                raise RuntimeError('injected observer startup failure')
    assert calls==[('get','power_save'),('set','power_save','off'),('set','power_save','on')]


def test_control_startup_requires_fresh_camera_before_guardian():
    import importlib,types,time
    with patch.dict(sys.modules,{'fcntl':types.ModuleType('fcntl')}):
        runtime=importlib.import_module('e6_observer.runtime')
    hub=runtime.Hub('fixture','unused','test',control_stage='shadow')
    with pytest.raises(RuntimeError,match='未就绪'):
        hub.wait_camera_ready(timeout=.02)
    assert hub.guardian is None
    hub.latest['rgb']={'x5_receive_monotonic_ns':time.perf_counter_ns()-501_000_000}
    with pytest.raises(RuntimeError,match='未就绪'):
        hub.wait_camera_ready(timeout=.02)
    hub.latest['rgb']={'x5_receive_monotonic_ns':time.perf_counter_ns()}
    hub.wait_camera_ready()
    assert hub.stats['control_startup']['camera_ready_before_guardian']
    hub.stats['rgb_error']='injected SDK failure'
    with pytest.raises(RuntimeError,match='injected SDK failure'):
        hub.wait_camera_ready()
    assert hub.guardian is None
