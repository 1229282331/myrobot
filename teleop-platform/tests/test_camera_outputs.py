import json
import threading
import time
from e6_phosphobot.camera_outputs import CameraOutputs,atomic
from e6_phosphobot.gateway import NativeExecutor


def wait_for(predicate):
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.005)
    assert predicate()


def test_blocked_snapshot_disk_does_not_block_capture_or_health(tmp_path):
    entered=threading.Event();release=threading.Event()
    def write(path,data):
        if path.name=='color.jpg':entered.set();assert release.wait(3)
        atomic(path,data)
    health=tmp_path/'memory/health.json'
    outputs=CameraOutputs(tmp_path/'disk',health,write=write);outputs.start()
    try:
        outputs.offer(dict(stream='color',index=1,host_perf_counter_ns=100),b'first',dict(rgb_received_ns=100,frame_id=1))
        assert entered.wait(1)
        for index in range(2,102):
            outputs.offer(dict(stream='color',index=index,host_perf_counter_ns=index),b'latest',dict(rgb_received_ns=index,frame_id=index))
        wait_for(lambda:health.exists() and json.loads(health.read_text())['frame_id']==101)
        assert json.loads(health.read_text())['rgb_received_ns']==101
        assert len(outputs.latest)==1
        release.set()
    finally:
        release.set();assert not outputs.close(dict(closed_ns=123,rgb_received_ns=101))
    assert json.loads(health.read_text())['closed_ns']==123


def test_snapshot_failure_keeps_independent_health_and_capture(tmp_path):
    def write(path,data):
        if path.name=='color.jpg':raise OSError('disk full')
        atomic(path,data)
    health=tmp_path/'memory/health.json';outputs=CameraOutputs(tmp_path/'disk',health,write=write);outputs.start()
    try:
        outputs.offer(dict(stream='color',index=1),b'jpg',dict(rgb_received_ns=123))
        wait_for(lambda:outputs.diagnostics['snapshot_error']=='disk full')
        outputs.offer(dict(stream='color',index=2),b'jpg2',dict(rgb_received_ns=456))
        wait_for(lambda:health.exists() and json.loads(health.read_text())['rgb_received_ns']==456)
        assert outputs.latest['color'][0]['index']==2
    finally:assert not outputs.close(dict(closed_ns=900))


def test_health_keeps_source_receipt_and_closed_camera_is_invalid(tmp_path):
    health=tmp_path/'health.json';now=1_000_000_000
    state=dict(serial='AY4A1520111',rgb_received_ns=now,frame_id=1)
    atomic(health,json.dumps(state).encode())
    executor=NativeExecutor('unused','shadow',health,clock=lambda:now+1)
    assert executor.camera()['valid']
    for extra in (dict(closed_ns=now),dict(error='PTS discontinuity')):
        atomic(health,json.dumps(dict(state,**extra)).encode())
        assert not executor.camera()['valid']


def test_health_close_publishes_terminal_state_even_if_previous_write_is_in_flight(tmp_path):
    entered=threading.Event();release=threading.Event();health=tmp_path/'health.json'
    def write(path,data):
        if path==health and not json.loads(data).get('closed_ns'):
            entered.set();assert release.wait(2)
        atomic(path,data)
    outputs=CameraOutputs(tmp_path/'disk',health,write=write);outputs.start()
    outputs.offer(dict(stream='color',index=1),b'jpg',dict(rgb_received_ns=1))
    assert entered.wait(1)
    closer=threading.Thread(target=lambda:outputs.close(dict(closed_ns=2,rgb_received_ns=1)))
    closer.start();wait_for(lambda:outputs.halt.is_set());release.set();closer.join(2)
    assert not closer.is_alive()
    assert json.loads(health.read_text())['closed_ns']==2


def test_live_snapshots_stay_in_session_memory_and_final_bytes_are_retained(tmp_path):
    disk=tmp_path/'disk';memory=tmp_path/'memory'
    outputs=CameraOutputs(disk,memory/'health.json',snapshot_root=memory/'snapshots',depth_valid=lambda *_:.75)
    outputs.start()
    try:
        outputs.offer(dict(stream='depth',index=12,source_received_perf_ns=123),b'\x01\x00',dict(depth_frame_id=12))
        outputs.offer(dict(stream='color',index=13,source_received_perf_ns=456),b'original-jpeg',dict(frame_id=13))
        wait_for(lambda:(memory/'snapshots/depth.json').exists() and (memory/'snapshots/color.json').exists())
        assert not list(disk.iterdir()), 'Live diagnostic images must not repeatedly write to the system disk'
        before={p.name:p.read_bytes() for p in (memory/'snapshots').iterdir()}
    finally:assert not outputs.close(dict(closed_ns=789))
    assert {p.name:p.read_bytes() for p in disk.iterdir()}==before
    assert json.loads((disk/'depth.json').read_text())['source_received_perf_ns']==123
    assert json.loads((disk/'depth.json').read_text())['valid_fraction']==.75
