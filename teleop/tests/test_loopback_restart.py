"""Linux loopback restart cannot take over an already listening workbench."""
import socket
import sys
import pytest
from e6_teleop.tunnel import ssh_loopback_forward


@pytest.mark.skipif(sys.platform!='linux',reason='Linux TIME_WAIT/reuse behavior')
def test_owned_linux_tunnel_restarts_but_rejects_an_active_owner():
    class Disconnected:
        def open_channel(self,*args):raise OSError('test disconnected')
    first=ssh_loopback_forward(Disconnected(),0)
    port=first.server_address[1]
    try:
        with pytest.raises(OSError):ssh_loopback_forward(Disconnected(),port)
        with socket.create_connection(('127.0.0.1',port),timeout=1) as client:
            assert client.recv(1)==b''
    finally:first.shutdown();first.server_close()
    second=ssh_loopback_forward(Disconnected(),port)
    second.shutdown();second.server_close()
