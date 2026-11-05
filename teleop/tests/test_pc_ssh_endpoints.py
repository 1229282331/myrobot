import struct
from unittest.mock import patch
import pytest
from e6_teleop.ssh_endpoints import observed_peer
from e6_teleop.socket_qos import ControlSocketQos


def test_nat_peer_is_selected_from_this_sessions_verified_server():
    assert observed_peer('192.168.1.7 49001 192.168.1.13 22',['192.168.1.13',22])==['192.168.1.7',49001]
    for connection in ('','192.168.1.7 49001 192.168.1.9 22',
                       '192.168.1.7 49001 192.168.1.13 23',
                       '8.8.8.8 49001 192.168.1.13 22',
                       '192.168.1.7 0 192.168.1.13 22'):
        with pytest.raises(ValueError):observed_peer(connection,['192.168.1.13',22])


def test_linux_owned_socket_marks_priority_and_reports_measured_rto():
    class Wire:
        tos=0
        def getsockname(self):return ('172.24.219.228',49001)
        def getpeername(self):return ('192.168.1.13',22)
        def setsockopt(self,level,option,value):self.tos=value
        def getsockopt(self,level,option,size=None):
            if size is None:return self.tos
            return bytes(8)+struct.pack('=I',70000)+bytes(92)
    with patch('e6_teleop.socket_qos.sys.platform','linux'),patch('e6_teleop.socket_qos.socket.TCP_INFO',11,create=True):
        qos=ControlSocketQos(Wire())
        assert qos.report['configured'] and qos.report['effective_ip_tos']==224
        assert qos.report['tcp_timing']['rto_ms']==70
        assert qos.report['tcp_timing']['route_minimum_is_not_actual_rto']
        qos.close()
