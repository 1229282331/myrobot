from fastapi.testclient import TestClient
import pytest
from e6_teleop.gateway import make_app
from e6_teleop.service import make_app as pc_app


def test_gateway_websocket_ownership_and_socket_session():
    app=make_app(render=False)
    with TestClient(app) as client:
        with client.websocket_connect("/ws/control") as ws:
            hello=ws.receive_json(); assert hello["ok"] and not hello["status"]["armed"]
            with client.websocket_connect("/ws/control") as second:
                assert second.receive_json()["ok"] is False
            ws.send_json(dict(operation="stop",session="wrong session"))
            assert not ws.receive_json()["ok"]
            status=hello["status"]
            ws.send_json(dict(operation="input",session=status["session"],lease=status["lease"],seq=1,
                              source="keyboard",identity="test",focused=True,connected=True,neutral=True,deadman=False,
                              stop=False,fine=False,frame="base",mode="translation",twist=[0]*6,gripper_rate=0,raw={}))
            assert ws.receive_json()["ok"]
            ws.send_json(dict(operation="arm",session=status["session"]))
            assert ws.receive_json()["status"]["armed"]
        assert app.state.core.owner is None and not app.state.core.armed
        with client.websocket_connect("/ws/control") as ws:
            status=ws.receive_json()["status"]
            assert not status["armed"]
            ws.send_json(dict(operation="arm",session=status["session"]))
            assert not ws.receive_json()["ok"]


def test_pc_rejects_remote_gateway_and_origin(tmp_path):
    with pytest.raises(ValueError,match="loopback"): pc_app(tmp_path,gateway="ws://192.168.1.4:8766")
    with TestClient(pc_app(tmp_path,gateway="ws://127.0.0.1:1")) as client:
        assert client.get("/api/state").json()["hardware_connected"] is False
        assert client.get("/api/state",headers={"Origin":"https://other.example"}).status_code==403
        assert client.get("/api/episodes/nonexistent").status_code==404
