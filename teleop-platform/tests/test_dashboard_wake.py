import socket
import threading
import pytest
from e6_phosphobot.dashboard import Dashboard,Busy
from e6_phosphobot.gateway import NativeExecutor


@pytest.mark.parametrize('reply', [b'0,{1},GetAngle();',b'-1,{},GetAngle();'])
def test_correlated_reply_wakes_after_candidate_wakeup_was_consumed(reply):
    peer,client=socket.socketpair();peer.settimeout(1)
    d=Dashboard('test');d._socket=client;d._connected=True
    wake=threading.Event();d.completion_wake=wake;d._start_receiver()
    try:
        p=d.submit('GetAngle()');assert peer.recv(1024)==b'GetAngle()\n'
        wake.set();assert wake.wait(.1);wake.clear()  # candidate arrived first
        assert not d.poll(p)
        with pytest.raises(Busy):d.submit('GetAngle()')
        peer.sendall(reply)
        assert wake.wait(1)
        assert p.done.is_set() and d.poll(p)
        assert p.event['accepted']==reply.startswith(b'0,')
        assert len(d.events)==1
    finally:d.disconnect();peer.close()


def test_connection_loss_wakes_pending_transaction_without_accepting_it():
    peer,client=socket.socketpair();peer.settimeout(1)
    d=Dashboard('test');d._socket=client;d._connected=True
    wake=threading.Event();d.completion_wake=wake;d._start_receiver()
    try:
        p=d.submit('GetAngle()');peer.recv(1024);peer.close()
        assert wake.wait(1) and p.done.is_set()
        assert not p.event['accepted']
        with pytest.raises(ConnectionError):d.poll(p)
    finally:d.disconnect()


def test_cpp_connection_binds_reply_wakeup_before_connect(monkeypatch):
    from types import SimpleNamespace
    import e6_phosphobot.gateway as gateway
    wake=threading.Event();seen=[]
    class Fake:
        def __init__(self,host):self.completion_wake=None
        def connect(self):seen.append(self.completion_wake)
    monkeypatch.setattr(gateway,'Dashboard',Fake)
    e=NativeExecutor('test','shadow',None)
    e.moveit=SimpleNamespace(initial_candidate_event=wake);e.moveit_api='cpp'
    e._connect_dashboard()
    assert seen==[wake]
