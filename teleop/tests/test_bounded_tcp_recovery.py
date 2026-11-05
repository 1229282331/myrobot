"""Unsupported/readback-mismatched kernels cannot silently bypass transport caps."""
import socket
from unittest.mock import patch
import pytest
from e6_teleop.socket_qos import linux_low_latency_recovery


class Wire:
    def __init__(self):self.options={}
    def setsockopt(self,level,option,value):
        assert level==socket.IPPROTO_TCP
        self.options[option]=value
    def getsockopt(self,level,option):return self.options[option]


def test_per_socket_floor_ack_budget_and_linear_recovery_are_verified():
    wire=Wire()
    with patch('e6_teleop.socket_qos.sys.platform','linux'):
        report=linux_low_latency_recovery(wire)
    assert wire.options=={45:20000,46:10000,16:1}
    assert report['min_rto_us']==20000 and report['delayed_ack_max_us']==10000
    assert not report['application_retry'] and not report['lease_changed']


@pytest.mark.parametrize('failure',['unsupported','wrong-readback'])
def test_unsupported_or_unapplied_settings_fail_closed(failure):
    wire=Wire()
    with patch('e6_teleop.socket_qos.sys.platform','linux'):
        if failure=='unsupported':
            with patch.object(wire,'setsockopt',side_effect=OSError('unsupported')):
                with pytest.raises(OSError):linux_low_latency_recovery(wire)
        else:
            with patch.object(wire,'getsockopt',return_value=300):
                with pytest.raises(ValueError):linux_low_latency_recovery(wire)
