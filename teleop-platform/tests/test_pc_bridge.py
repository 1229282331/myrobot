import pytest
from fastapi import FastAPI,HTTPException
from fastapi.testclient import TestClient
from e6_phosphobot.pc_bridge import StateCache,make_display_robot,install_hardware


def test_real_adapter_exposes_no_legacy_device_write_paths():
    cache=StateCache()
    robot=make_display_robot(cache)
    for method,args in [('set_motors_positions',([0]*6,)),('enable_torque',()),
                        ('disable_torque',()),('init_config',())]:
        with pytest.raises(HTTPException) as error:getattr(robot,method)(*args)
        assert error.value.status_code==409
    with pytest.raises(RuntimeError):robot.get_observation()
    with pytest.raises(HTTPException):robot.read_joints_position()


def test_legacy_routes_blocked_before_upstream_handlers():
    app=FastAPI()
    install_hardware(app,'ws://127.0.0.1:8776')
    # No lifespan/devices started. These requests must stop at the adapter.
    client=TestClient(app)
    for path in ('/move/init','/move/relative','/move/absolute','/move/sleep','/joints/write'):
        response=client.post(path,json={})
        assert response.status_code==409,(path,response.text)


def test_bridge_requires_local_tunnel():
    with pytest.raises(ValueError):install_hardware(FastAPI(),'ws://192.168.1.13:8776')


def test_native_record_browser_available_without_device_observation(tmp_path):
    app=FastAPI();install_hardware(app,'ws://127.0.0.1:8776',episodes_dir=tmp_path)
    client=TestClient(app)
    assert client.get('/e6/capabilities').json()['original_recording']
    assert client.get('/e6/episodes').json()['episodes']==[]
    assert client.get('/e6/state').status_code==503


def test_video_clock_probe_does_not_replace_actual_device_state():
    cache=StateCache();state=dict(kind='state',actual=dict(gripper=None))
    assert cache.update(state)
    received=cache.received_ns
    assert not cache.update(dict(kind='clock_probe',client_sent_ms=1,x5_received_ns=2))
    assert cache.current()==state and cache.received_ns==received


@pytest.mark.parametrize('clean',[True,False])
def test_refusal_close_preserves_reason_but_reports_actual_transport_failure(monkeypatch,clean):
    import asyncio,json
    from starlette.websockets import WebSocketDisconnect
    from websockets.exceptions import ConnectionClosedOK,ConnectionClosedError
    from websockets.frames import Close
    from e6_phosphobot import pc_bridge

    class Refused:
        async def __aenter__(self):return self
        async def __aexit__(self,*_):pass
        async def send(self,raw):
            if json.loads(raw).get('role')=='control':return
            if not clean:raise ConnectionClosedError(None,None)
            raise ConnectionClosedOK(Close(1000,''),Close(1000,''),True)
        async def __aiter__(self):
            yield json.dumps(dict(kind='state',ready=False,reason='已有控制者'))
            await asyncio.Event().wait()

    monkeypatch.setattr(pc_bridge,'connect',lambda *a,**kw:Refused())
    app=FastAPI();install_hardware(app,'ws://127.0.0.1:8776')
    with TestClient(app).websocket_connect('/e6/control') as socket:
        assert socket.receive_json()['reason']=='已有控制者'
        socket.send_json(dict(op='intent'))
        if not clean:
            assert socket.receive_json()['reason'].startswith('X5连接已断开:')
        with pytest.raises(WebSocketDisconnect):socket.receive_json()
    trace=TestClient(app).get('/e6/diagnostics/input-relay').json()
    assert trace['total']==1 and trace['overwritten']==0
    assert trace['rows'][0]['outcome']=='interrupted'
    assert trace['rows'][0]['pc_send_finished_ns']>=trace['rows'][0]['pc_received_ns']
