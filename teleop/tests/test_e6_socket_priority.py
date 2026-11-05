"""Owned socket requests are read back; they do not prove radio scheduling."""
import socket
import pytest
from e6_observer.control_guardian import DashboardLink, configure_e6_socket_priority


class Wire:
    def __init__(self, mismatch=None, fail=False):
        self.options={};self.closed=False;self.mismatch=mismatch;self.fail=fail
    def setsockopt(self, level, option, value):
        if self.fail and option==socket.SO_PRIORITY:raise OSError('priority unavailable')
        self.options[level,option]=value
    def getsockopt(self, level, option):
        return 0 if option==self.mismatch else self.options[level,option]
    def close(self):self.closed=True


def test_explicit_up7_requests_and_readbacks_without_wire_claim(monkeypatch):
    monkeypatch.setattr(socket,'SO_PRIORITY',12,raising=False)
    wire=Wire();result=configure_e6_socket_priority(wire)
    assert result['effective_ip_tos']==224
    assert result['effective_socket_priority']==263
    assert not result['wire_80211_priority_verified']


@pytest.mark.parametrize('failure',['tos','priority','error'])
def test_failed_request_closes_owned_dashboard_before_any_query(monkeypatch,failure):
    monkeypatch.setattr(socket,'SO_PRIORITY',12,raising=False)
    wire=Wire(mismatch=socket.IP_TOS if failure=='tos' else socket.SO_PRIORITY if failure=='priority' else None,
              fail=failure=='error')
    monkeypatch.setattr(socket,'create_connection',lambda *_args,**_kwargs:wire)
    with pytest.raises((ValueError,OSError)):DashboardLink('127.0.0.1')
    assert wire.closed


def test_platform_without_linux_priority_reports_only_tos(monkeypatch):
    monkeypatch.delattr(socket,'SO_PRIORITY',raising=False)
    result=configure_e6_socket_priority(Wire())
    assert result['effective_ip_tos']==224
    assert result['effective_socket_priority'] is None
    assert not result['wire_80211_priority_verified']
