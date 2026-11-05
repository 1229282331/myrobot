import io
import json
import sys
from types import SimpleNamespace
import pytest
from e6_phosphobot import camera_service


def test_unknown_preview_codec_fails_before_camera_or_output_creation(tmp_path):
    from e6_phosphobot.camera_service import run
    with pytest.raises(ValueError,match='Unknown preview codec'):
        run(SimpleNamespace(preview_codec='unrecognized',rtsp_url='rtsp://unused',output=tmp_path/'unused'))
    assert not (tmp_path/'unused').exists()


@pytest.mark.parametrize('basis,prefix',[('uvc_kernel_receipt','E6K'),('sdk_wait_return','E6S')])
def test_marker_retains_actual_basis_epoch_and_source_time_without_mutating_metadata(basis,prefix):
    epoch='1234567890abcdef'*2
    meta=dict(source_receipt_basis=basis,capture_epoch=epoch,index=23,
              source_received_perf_ns=1050123456,host_perf_counter_ns=1099123456)
    original=meta.copy()
    assert camera_service.latency_marker_payload(meta)==f'{prefix}:{epoch}:23:1050'
    assert meta==original


def test_parallel_marker_is_submitted_before_decode_and_matches_each_frame():
    from concurrent.futures import ThreadPoolExecutor
    import threading
    import cv2
    import numpy as np
    decoded=threading.Event();encoder=cv2.QRCodeEncoder_create();payloads=[]
    class Encoder:
        def encode(self,payload):
            assert decoded.wait(1), 'JPEG decoding should overlap QR generation'
            payloads.append(payload);return encoder.encode(payload)
    def decode(data,width,height,fmt):
        decoded.set();return np.full((height,width,3),int(data),np.uint8)
    with ThreadPoolExecutor(max_workers=1) as pool:
        for index in (8,9):
            decoded.clear()
            meta=dict(index=index,width=1280,height=720,format_id=5,host_perf_counter_ns=2000000000,
                source_received_perf_ns=1050123456,source_receipt_basis='uvc_kernel_receipt',capture_epoch='1'*32)
            image=camera_service.render_preview(meta,str(index),decode,cv2,Encoder(),pool)
            assert image.shape==(360,640,3) and np.all(image[-1,0]==index)
            value,_,_=cv2.QRCodeDetector().detectAndDecode(image[:132,508:640])
            assert value==camera_service.latency_marker_payload(meta)
    assert len(payloads)==2 and payloads[0]!=payloads[1]


def test_marker_failure_is_propagated_and_not_replaced_by_an_old_marker():
    from concurrent.futures import ThreadPoolExecutor
    import cv2
    import numpy as np
    class Encoder:
        def encode(self,payload):raise RuntimeError('QR job failed')
    meta=dict(index=8,width=1280,height=720,format_id=5,host_perf_counter_ns=2000000000,
        source_received_perf_ns=1050123456,source_receipt_basis='uvc_kernel_receipt',capture_epoch='1'*32)
    with ThreadPoolExecutor(max_workers=1) as pool:
        with pytest.raises(RuntimeError,match='QR job failed'):
            camera_service.render_preview(meta,b'',lambda *args:np.zeros((720,1280,3),np.uint8),cv2,Encoder(),pool)


def args(tmp_path,preview):
    return SimpleNamespace(camera_tools=tmp_path,output=tmp_path/'capture',health_file=tmp_path/'memory/health.json',
                           rtsp_url='rtsp://unused' if preview else None,latency_marker=False)


def test_sdk_start_failure_closes_encoder_before_worker_ever_started(tmp_path,monkeypatch):
    class Broken:
        def __init__(self,*a,**k):raise RuntimeError('SDK startup failed')
    encoder=SimpleNamespace(stdin=io.BytesIO(),terminate=lambda:None,wait=lambda timeout=None:0)
    monkeypatch.setattr(camera_service.subprocess,'Popen',lambda *a,**k:encoder)
    monkeypatch.setattr(camera_service.signal,'signal',lambda *a:None)
    monkeypatch.setattr(camera_service.glob,'glob',lambda *a:[])
    monkeypatch.setitem(sys.modules,'camera',SimpleNamespace(Camera=Broken,decode_color=lambda *a:None,decode_depth=lambda *a:None))
    configuration=args(tmp_path,True)
    with pytest.raises(RuntimeError,match='SDK startup failed'):camera_service.run(configuration)
    final=json.loads((configuration.output/'final.json').read_text())
    assert final['error']=='SDK startup failed' and final['closed_ns']
    assert final['cleanup_errors']==[] and encoder.stdin.closed
    assert json.loads(configuration.health_file.read_text())['closed_ns']


def test_sdk_cleanup_returned_errors_are_preserved(tmp_path,monkeypatch):
    class Broken:
        def __init__(self,*a,**k):self.info={};self.selected={}
        def read(self):raise RuntimeError('bad PTS')
        def close(self):return ['device release failed']
    monkeypatch.setattr(camera_service.signal,'signal',lambda *a:None)
    monkeypatch.setattr(camera_service.glob,'glob',lambda *a:[])
    monkeypatch.setitem(sys.modules,'camera',SimpleNamespace(Camera=Broken,decode_color=lambda *a:None,decode_depth=lambda *a:None))
    configuration=args(tmp_path,False)
    with pytest.raises(RuntimeError,match='bad PTS'):camera_service.run(configuration)
    final=json.loads((configuration.output/'final.json').read_text())
    assert final['error']=='bad PTS' and final['cleanup_errors']==['camera: device release failed']


def test_explicit_restart_reuses_owned_directory_without_overwriting_media(tmp_path,monkeypatch):
    configuration=args(tmp_path,True);configuration.resume_output=True
    configuration.output.mkdir()
    (configuration.output/'ffmpeg.log').write_bytes(b'previous encoder evidence\n')
    original=configuration.output/'media'/'prior'/'frame.raw'
    original.parent.mkdir(parents=True);original.write_bytes(b'original')
    reached=[]
    class Broken:
        def __init__(self,*a,**k):
            reached.append(True);raise RuntimeError('SDK startup reached')
    encoder=SimpleNamespace(stdin=io.BytesIO(),terminate=lambda:None,wait=lambda timeout=None:0)
    monkeypatch.setattr(camera_service.subprocess,'Popen',lambda *a,**k:encoder)
    monkeypatch.setattr(camera_service.signal,'signal',lambda *a:None)
    monkeypatch.setattr(camera_service.glob,'glob',lambda *a:[])
    monkeypatch.setitem(sys.modules,'camera',SimpleNamespace(Camera=Broken,decode_color=lambda *a:None,decode_depth=lambda *a:None))
    with pytest.raises(RuntimeError,match='SDK startup reached'):camera_service.run(configuration)
    assert reached==[True] and original.read_bytes()==b'original'
    assert (configuration.output/'ffmpeg.log').read_bytes()==b'previous encoder evidence\n'


def test_existing_output_is_rejected_unless_explicit_restart(tmp_path):
    with pytest.raises(FileExistsError):camera_service.prepare_output(tmp_path)


@pytest.mark.parametrize('name',['recording.request.json','final.json'])
def test_restart_requires_preserving_previous_control_and_status(tmp_path,name):
    (tmp_path/name).write_text('prior evidence')
    with pytest.raises(ValueError,match='archived prior'):camera_service.prepare_output(tmp_path,True)
    assert (tmp_path/name).read_text()=='prior evidence'
