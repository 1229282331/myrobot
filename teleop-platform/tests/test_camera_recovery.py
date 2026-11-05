import importlib.util
from pathlib import Path
from types import SimpleNamespace
import struct
import pytest
from e6_phosphobot.camera_recovery import RecoveringCapture,source_receipt
from e6_phosphobot.session_runner import check_children

spec=importlib.util.spec_from_file_location('retained_uvc_clock',Path(__file__).parents[2]/'dcw/tools/uvc_clock.py')
uvc=importlib.util.module_from_spec(spec);spec.loader.exec_module(uvc)


def test_pause_discards_old_frames_resumes_with_explicit_pts_epoch_and_original_fields():
    now=[10_000_000_000];events=[]
    class Device:
        def __init__(self):self.uvc_clock=uvc.PtsClock(15_000_000);self.receipt=now[0]-20_000_000;self.pts=1000
        def read(self):
            raw=struct.pack('<QHBBI',self.receipt,0,6,4,self.pts)
            pts,kernel=uvc.v4l2_pts(raw,now[0])
            meta=dict(stream='color',index=self.pts,host_monotonic_ns=now[0],
                      host_perf_counter_ns=now[0]+100,uvc_kernel_receive_ns=kernel,
                      device_timestamp_us=987654,**self.uvc_clock.add(pts))
            return [(meta,b'original-jpeg')]
    d=Device();c=RecoveringCapture(d,clock=lambda:now[0]+100,report=events.append)
    first=c.read()[0][0];assert first['uvc_pts_unwrapped']==1000
    now[0]+=3_200_000_000
    assert c.read()==[] and c.recovering
    assert c.stats['rejected_framesets']==1
    # Old bytes may not refresh acquisition or enter a dataset.
    d.receipt=now[0]-10_000_000;d.pts=1000+15_000_000*4
    fresh,data=c.read()[0]
    assert data==b'original-jpeg' and fresh['capture_epoch']!=first['capture_epoch']
    assert fresh['uvc_pts_unwrapped']==d.pts and fresh['uvc_pts_wrap_count']==0
    assert fresh['device_timestamp_us']==987654 and fresh['host_perf_counter_ns']==now[0]+100
    assert fresh['source_received_perf_ns']==d.receipt+100
    assert c.stats['capture_recoveries']==1 and not c.recovering
    assert [e['event'] for e in events]==['capture_gap','capture_recovered']


def test_sub_three_second_buffered_frame_is_dropped_and_missing_rgb_cannot_end_quarantine():
    now=[1_000_000_000]
    def frame(stream='color',kernel=400_000_000):
        return [(dict(stream=stream,host_perf_counter_ns=now[0],host_monotonic_ns=now[0],
                      uvc_kernel_receive_ns=kernel),b'bytes')]
    d=SimpleNamespace(uvc_clock=uvc.PtsClock(15_000_000),read=lambda:frame())
    c=RecoveringCapture(d,clock=lambda:now[0]);assert c.read()==[]
    for stream in ('depth','color'):
        d.read=lambda:frame(stream,None)
        assert c.read()==[] and c.recovering
    d.read=lambda:frame('color',now[0]-10_000_000)
    assert len(c.read())==1 and not c.recovering


def test_poll_gap_without_pts_is_not_mistaken_for_fresh_capture():
    now=[1_000_000_000]
    d=SimpleNamespace(uvc_clock=uvc.PtsClock(15_000_000),read=lambda:[(dict(stream='color',host_perf_counter_ns=now[0]),b'data')])
    c=RecoveringCapture(d,clock=lambda:now[0]);assert c.read()
    now[0]+=600_000_000
    assert c.read()==[] and c.stats['capture_poll_gaps']==1


@pytest.mark.parametrize('message',['Truncated Linux UVC metadata block','UVC packet reports a payload error'])
def test_structurally_bad_uvc_is_not_silently_accepted(message):
    def read():raise ValueError(message)
    c=RecoveringCapture(SimpleNamespace(read=read))
    with pytest.raises(ValueError,match=message):c.read()


def test_same_host_source_receipt_uses_measured_clock_offset_without_mutating_metadata():
    m=dict(host_monotonic_ns=1000,host_perf_counter_ns=2000,uvc_kernel_receive_ns=900)
    assert source_receipt(m)==(1900,'uvc_kernel_receipt')
    assert m['host_perf_counter_ns']==2000
    with pytest.raises(ValueError):source_receipt(dict(m,uvc_kernel_receive_ns=1001))


def test_camera_child_exit_keeps_feedback_owner_and_logs_once():
    camera=SimpleNamespace(pid=11,poll=lambda:1);gateway=SimpleNamespace(pid=12,poll=lambda:None)
    result={};events=[]
    for _ in range(3):check_children(dict(camera_service=camera,gateway=gateway),result,lambda *a,**k:events.append((a,k)))
    assert len(events)==1 and result['camera_failure']==dict(pid=11,exit_code=1)
    gateway.poll=lambda:2
    with pytest.raises(RuntimeError,match='gateway exited'):check_children(dict(camera_service=camera,gateway=gateway),result,lambda *a,**k:None)
