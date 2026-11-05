"""Validate native bytes/metadata and ordered, bounded preview workers."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
from pathlib import Path
import queue
import sys
import threading
import time
import cv2
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'dcw/tools'))
from e6_observer.camera_frames import build_sample
from camera import decode_color,decode_depth
from rdk_healthcheck import ordered_frames

PROFILE=dict(rgb=[4,2],depth=[4,2],jpeg_quality=55)


def sample(stream,index=1):
    meta=dict(stream=stream,width=8,height=4,index=index,host_perf_counter_ns=123456+index,
              format_id=5 if stream=='color' else 12,depth_scale_mm=1.,uvc_pts_unwrapped=index)
    if stream=='color':
        ok,encoded=cv2.imencode('.jpg',np.full((4,8,3),128,dtype=np.uint8));assert ok
        raw=encoded.tobytes()
    else:raw=(np.arange(32,dtype='<u2')*100).tobytes()
    return meta,raw


@pytest.mark.parametrize('stream',['color','depth'])
def test_native_bytes_and_frame_identity_survive_worker_processing(stream):
    meta,raw=sample(stream);original=dict(meta)
    value=build_sample(meta,raw,PROFILE,decode_color,decode_depth)
    assert meta==original and value['metadata']==meta
    assert value['x5_receive_monotonic_ns']==meta['host_perf_counter_ns']
    assert value['device_frame_id']==meta['index']
    assert value['original_sha256']==hashlib.sha256(raw).hexdigest()
    assert value['jpeg' if stream=='color' else 'uint16_raw']==raw
    assert cv2.imdecode(np.frombuffer(value['preview_jpeg'],np.uint8),cv2.IMREAD_UNCHANGED).shape[:2]==(2,4)
    if stream=='depth':assert value['valid_fraction']==31/32 and value['data_valid']


def test_invalid_rgb_and_depth_fail_without_creating_a_sample():
    for stream in ('color','depth'):
        meta,raw=sample(stream)
        with pytest.raises(ValueError):build_sample(meta,raw[:-20],PROFILE,decode_color,decode_depth)
    meta,raw=sample('depth')
    zero=build_sample(meta,bytes(len(raw)),PROFILE,decode_color,decode_depth)
    assert zero['uint16_raw']==bytes(len(raw)) and not zero['data_valid']


def test_parallel_preview_workers_emit_reception_order_and_remain_bounded():
    inbox=queue.Queue(24);done=threading.Event();done.set();report=dict(decode_pending_max=0)
    for index in range(8):inbox.put(sample('color' if index%2==0 else 'depth',index))
    completed=[]
    def worker(meta,raw,decode_rgb=True):
        begin=time.perf_counter_ns()
        if meta['index']==0:time.sleep(.04)
        value=build_sample(meta,raw,PROFILE,decode_color,decode_depth)
        completed.append(meta['index'])
        return value,begin,time.perf_counter_ns()
    with ThreadPoolExecutor(max_workers=2) as pool:
        results=list(ordered_frames(inbox,done,pool,2,report,1,decoder=worker))
    assert completed[0]==1 and [value['device_frame_id'] for _,value,_,_ in results]==list(range(8))
    assert report['decode_pending_max']<=4 and len(results)==8
