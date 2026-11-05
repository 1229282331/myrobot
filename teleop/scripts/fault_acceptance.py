"""Inject PC recording failure against an already running loopback simulator."""
import json
from pathlib import Path
import tempfile
import time
from unittest.mock import patch
from fastapi.testclient import TestClient
from e6_teleop.service import make_app
from e6_teleop.recording import EpisodeRecorder


def fail_sample(self, frame): raise OSError("injected recording disk failure")


with tempfile.TemporaryDirectory(prefix="e6-simulation-fault-") as directory:
    with patch.object(EpisodeRecorder,"sample",fail_sample):
        with TestClient(make_app(directory)) as client:
            for _ in range(100):
                if client.get("/api/state").json()["gateway_ready"]: break
                time.sleep(.05)
            else: raise RuntimeError("Start the local simulation gateway first")
            with client.websocket_connect("/ws/control") as ws:
                status=ws.receive_json()["status"]
                ws.send_json(dict(operation="input",lease=status["lease"],seq=1,source="keyboard",identity="disk fault test",
                                  connected=True,focused=True,deadman=False,neutral=True,stop=False,fine=False,
                                  frame="base",mode="translation",twist=[0]*6,gripper_rate=0,raw={}))
                assert ws.receive_json()["ok"]
                ws.send_json(dict(operation="arm"));assert ws.receive_json()["status"]["armed"]
                ws.send_json(dict(operation="record/start",task="模拟录制故障注入"));assert ws.receive_json()["ok"]
                for _ in range(40):
                    episodes=client.get("/api/episodes").json()
                    if episodes: break
                    time.sleep(.05)
                assert episodes[0]["result"]=="aborted" and episodes[0]["complete"] is False
                assert "injected recording disk failure" in episodes[0]["reason"]
                ws.send_json(dict(operation="stop"));status=ws.receive_json()["status"]
                assert not status["armed"]
                assert client.post(f"/api/export/{episodes[0]['id']}").status_code==400
                report=dict(recording_failure_aborts=True,stop_requires_rearm=True,incomplete_export_rejected=True,
                            reason=episodes[0]["reason"],hardware_connections=False)
    evidence=Path("docs/evidence/teleop_simulation_20261004");evidence.mkdir(parents=True,exist_ok=True)
    (evidence/"recording_fault.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,indent=2))
