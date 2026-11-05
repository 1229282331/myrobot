"""The return-flow setting may select only its own authenticated SSH tuple."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.ssh_socket_qos import session_endpoints,configure
from e6_teleop.observation_service import make_app


def test_expected_tuple_and_ssh_environment_must_agree():
    assert session_endpoints(['192.168.1.7',41234],'192.168.1.7 41234 192.168.1.13 22')==(['192.168.1.7',41234],['192.168.1.13',22])
    for value in ('','192.168.1.8 41234 192.168.1.13 22','192.168.1.7 41235 192.168.1.13 22','192.168.1.7 0 192.168.1.13 22','192.168.1.7 41234 192.168.1.13 65536'):
        with pytest.raises(ValueError):session_endpoints(['192.168.1.7',41234],value)


def test_no_ssh_environment_refuses_before_native_descriptor_access(monkeypatch):
    monkeypatch.delenv('SSH_CONNECTION',raising=False)
    with pytest.raises(ValueError,match='owned SSH_CONNECTION'):configure(['192.168.1.7',41234])


def test_preview_may_use_distinct_loopback_transport_but_not_remote_listener(tmp_path):
    app=make_app(tmp_path,gateway='ws://127.0.0.1:8771',media_gateway='ws://127.0.0.1:8767',render=False)
    assert app is not None
    with pytest.raises(ValueError,match='loopback media'):
        make_app(tmp_path,media_gateway='ws://192.168.1.13:8767',render=False)
