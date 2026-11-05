import threading
import socket
import pytest
from e6_phosphobot import launcher


class Stream:
    def __init__(self, error=None):self.error=error;self.closes=0
    def settimeout(self, _):pass
    def recv(self, _):return b''
    def close(self):
        self.closes+=1
        if self.error:raise self.error


def tunnel():
    result=launcher.Tunnels.__new__(launcher.Tunnels)
    result.active=set();result.lock=threading.Lock();result.halt=threading.Event()
    result.servers=[];result.clients=[]
    return result


def test_peer_eof_during_relay_cleanup_releases_both_ownership_entries(monkeypatch):
    owner=tunnel();a=Stream();b=Stream(EOFError())
    monkeypatch.setattr(launcher.select,'select',lambda *args:([a],[],[]))
    owner.relay(a,b)
    assert a.closes==1 and b.closes==1
    assert not owner.active


def test_owner_shutdown_allows_relay_cleanup_without_lock_or_eof_escape():
    owner=tunnel();cleanup_entered=threading.Event()
    class ClosingStream(Stream):
        def close(self):
            # A close implementation may wait for the relay to release ownership.
            with owner.lock:cleanup_entered.set()
            super().close()
    a=ClosingStream(EOFError());b=Stream(OSError('socket closed'));client=Stream()
    owner.active.update((a,b));owner.clients=[client]
    task=threading.Thread(target=owner.close,daemon=True);task.start();task.join(1)
    assert not task.is_alive() and cleanup_entered.is_set()
    assert a.closes==b.closes==client.closes==1
    assert owner.halt.is_set()


def test_repeated_disconnected_close_is_idempotent():
    stream=Stream(EOFError())
    launcher.Tunnels.close_stream(stream)
    launcher.Tunnels.close_stream(stream)
    assert stream.closes==2


def test_only_control_transport_disables_nagle_and_reports_readback():
    class Socket:
        def __init__(self):self.value=0;self.writes=[]
        def getsockopt(self,*_):return self.value
        def setsockopt(self,level,option,value):
            assert (level,option)==(socket.IPPROTO_TCP,socket.TCP_NODELAY)
            self.writes.append(value);self.value=value
    class Client(Stream):
        def __init__(self):super().__init__();self.sock=Socket()
        def get_transport(self):return self
    clients=[Client() for _ in range(3)]
    owner=launcher.Tunnels(iter(clients).__next__)
    assert owner.control_network==dict(tcp_nodelay=1,previous_tcp_nodelay=0)
    assert [c.sock.writes for c in clients]==[[1],[],[]]
    owner.close()
    assert all(c.closes==1 for c in clients)


def test_socket_option_failure_releases_all_opened_transports():
    class BadSocket:
        def getsockopt(self,*_):return 0
        def setsockopt(self,*_):raise OSError('unsupported')
    class Client(Stream):
        sock=BadSocket()
        def get_transport(self):return self
    clients=[Client() for _ in range(3)]
    with pytest.raises(OSError,match='unsupported'):
        launcher.Tunnels(iter(clients).__next__)
    assert all(c.closes==1 for c in clients)
