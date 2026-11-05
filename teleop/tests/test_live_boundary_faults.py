"""Boundary fault injection must affect only the recorded control SSH owner."""
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from live_ssh_fault_acceptance import cut_owned_socket


class Client:
    def __init__(self,pid=1234,admin_port=55000):self.pid=pid;self.admin_port=admin_port;self.commands=[];self.cut=False;self.closed=False
    def load_system_host_keys(self):pass
    def set_missing_host_key_policy(self,_):pass
    def connect(self,*a,**kw):pass
    def get_transport(self):return SimpleNamespace(sock=SimpleNamespace(getsockname=lambda:('192.168.1.7',self.admin_port)))
    def exec_command(self,command):
        self.commands.append(command)
        if command.startswith('ss -Ktn'):self.cut=True;value='socket destroyed\n'
        elif command.startswith('ss -Htnp'):
            value='' if self.cut else f'0 0 192.168.1.13:22 192.168.1.7:63741 users:(("sshd",pid={self.pid},fd=4))\n'
        else:raise AssertionError('Unexpected remote operation: '+command)
        out=io.BytesIO(value.encode());out.channel=SimpleNamespace(recv_exit_status=lambda:0)
        return None,out,io.BytesIO()
    def close(self):self.closed=True


def args(**kw):return SimpleNamespace(**dict(host='X5',user='root',peer='192.168.1.7',peer_port=63741,sshd_pid=1234,**kw))


def test_only_matching_recorded_socket_can_be_destroyed(monkeypatch):
    c=Client();monkeypatch.setattr('live_ssh_fault_acceptance.paramiko.SSHClient',lambda:c)
    result=cut_owned_socket(args())
    assert result['after']=='' and c.closed
    assert [s for s in c.commands if s.startswith('ss -K')]==['ss -Ktn dst 192.168.1.7 dport = :63741 sport = :22']


@pytest.mark.parametrize('client',[Client(pid=999),Client(admin_port=63741)])
def test_changed_owner_and_diagnostic_socket_are_never_cut(monkeypatch,client):
    monkeypatch.setattr('live_ssh_fault_acceptance.paramiko.SSHClient',lambda:client)
    with pytest.raises(ValueError):cut_owned_socket(args())
    assert client.closed and not client.cut
