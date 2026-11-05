import json
from types import SimpleNamespace
import pytest
from e6_phosphobot.camera_archive import atomic_json
from e6_phosphobot.camera_restart import CameraRestart,CameraRestartSupervisor
from test_gateway import setup


def test_resource_supervisor_import_needs_no_camera_or_numerical_dependencies():
    from pathlib import Path
    import subprocess,sys
    source=str(Path(__file__).resolve().parents[1]/'src')
    code='import sys;sys.path.insert(0,'+repr(source)+');from e6_phosphobot.camera_restart import CameraRestartSupervisor'
    subprocess.run([sys.executable,'-S','-c',code],check=True,capture_output=True)


def harness(tmp_path):
    now=[1_000_000_000];calls=[];events=[];alive=[True]
    path=tmp_path/'health.json'
    atomic_json(path,dict(serial='AY4A1520111',capture_epoch='old',rgb_received_ns=now[0]))
    client=CameraRestart(path,'session-one',clock=lambda:now[0])
    def lifecycle(kind,**values):
        events.append(dict(event=kind,at_ns=now[0],**values))
    supervisor=CameraRestartSupervisor(path,'session-one',calls.append,lambda:alive[0],
        lifecycle,clock=lambda:now[0])
    return client,supervisor,now,calls,events,alive


def test_camera_restart_requires_new_epoch_and_source_time_never_replays_request(tmp_path):
    c,s,now,calls,events,alive=harness(tmp_path)
    identity=c.request();s.step();assert calls==[identity]
    s.step();assert c.status()['phase']=='starting'  # Old healthy image cannot complete recovery.
    now[0]+=50_000_000
    atomic_json(c.health,dict(serial='AY4A1520111',capture_epoch='new',rgb_received_ns=now[0]))
    s.step();assert c.status()['phase']=='complete'
    for _ in range(3):s.step()
    assert calls==[identity]
    assert [v['phase'] for v in events]==['starting','complete']
    assert all(v['at_ns']==v['status_at_ns'] for v in events)


@pytest.mark.parametrize('received',[None,0,'starting',True,float('nan')])
def test_new_capture_without_first_rgb_waits_instead_of_raising(tmp_path,received):
    c,s,now,calls,events,alive=harness(tmp_path)
    identity=c.request();s.step()
    atomic_json(c.health,dict(serial='AY4A1520111',capture_epoch='new',rgb_received_ns=received))
    s.step();assert c.status()['phase']=='starting' and calls==[identity]
    now[0]+=1_000_000
    atomic_json(c.health,dict(serial='AY4A1520111',capture_epoch='new',rgb_received_ns=now[0]))
    s.step();assert c.status()['phase']=='complete'


@pytest.mark.parametrize('failure',['exception','exit','no_new_image'])
def test_restart_failure_preserves_reason_and_requires_another_explicit_request(tmp_path,failure):
    c,s,now,calls,events,alive=harness(tmp_path)
    if failure=='exception':
        def broken(_):raise OSError('camera device busy')
        s.restart=broken
    c.request();s.step()
    if failure=='exit':alive[0]=False
    if failure=='no_new_image':now[0]+=15_000_000_001
    s.step();assert c.status()['phase']=='failed' and c.status()['error']
    before=len(calls)
    for _ in range(3):s.step()
    assert len(calls)==before


@pytest.mark.parametrize('change',[{'session':'other-session'},{'requested_ns':0}])
def test_wrong_session_or_old_restart_never_reaches_child(tmp_path,change):
    c,s,now,calls,_,_=harness(tmp_path)
    c.request();now[0]+=6_000_000_000
    data=json.loads(c.request_path.read_text());data.update(change);atomic_json(c.request_path,data)
    s.step();assert calls==[] and c.status()['phase']=='failed'


def test_gateway_camera_restart_stops_readiness_and_requires_explicit_prepare(tmp_path):
    e,d,send,step,now=setup()
    e.camera_restart=CameraRestart(tmp_path/'health.json','one',clock=lambda:now[0])
    send('camera_restart')
    assert e.camera_restart.request_path.exists() and not e.session.ready and not d.calls
    send();send('prepare');assert not e.session.ready
    atomic_json(e.camera_restart.status_path,dict(session='one',id=e.camera_restart.pending,phase='complete'))
    step();assert not e.session.ready
    send();send('prepare');assert e.session.ready and not d.calls


@pytest.mark.parametrize('condition',['moving','pending','stopping','stale','recording','sealing','transfer','deadman'])
def test_gateway_restart_cannot_interrupt_motion_recording_or_accept_unreleased_input(tmp_path,condition):
    e,d,send,step,now=setup()
    e.camera_restart=CameraRestart(tmp_path/'health.json','one',clock=lambda:now[0])
    if condition=='moving':e.moving=True
    if condition=='pending':e.pending=object()
    if condition=='stopping':e.stop_pending=object()
    if condition=='stale':d.data['age_ms']=200
    if condition in ('recording','sealing','transfer'):
        phase={'recording':'capturing','sealing':'sealing','transfer':'transfer_pending'}[condition]
        e.episodes=SimpleNamespace(state=lambda:dict(phase=phase))
    if condition=='deadman':send(deadman=True)
    send('camera_restart')
    assert not e.camera_restart.request_path.exists() and not d.calls
