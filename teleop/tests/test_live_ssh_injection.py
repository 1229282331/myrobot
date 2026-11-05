"""Fault injection must never terminate an unowned or master SSH process."""
import io,shlex,sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import live_ssh_fault_acceptance as injection


class Client:
    def __init__(self,rows,process='sshd: root@notty'):
        self.rows=iter(rows);self.process=process;self.commands=[];self.closed=False
    def load_system_host_keys(self):pass
    def set_missing_host_key_policy(self,_):pass
    def connect(self,*args,**kwargs):pass
    def get_transport(self):
        return SimpleNamespace(sock=SimpleNamespace(getsockname=lambda:('192.168.1.7',60001)))
    def exec_command(self,text):
        args=shlex.split(text);self.commands.append(args)
        result=next(self.rows) if args[:2]==['ss','-Htnp'] else self.process if args[0]=='ps' else ''
        out=io.BytesIO(result.encode());out.channel=SimpleNamespace(recv_exit_status=lambda:0)
        return None,out,io.BytesIO(b'')
    def close(self):self.closed=True


ROW='0 0 192.168.1.13:22 192.168.1.7:60874 users:(("sshd",pid=49000,fd=4))\n'
ARGS=SimpleNamespace(peer='192.168.1.7',peer_port=60874,sshd_pid=49000,host='192.168.1.13',user='root')


def use(monkeypatch,client):
    monkeypatch.setattr(injection.paramiko,'SSHClient',lambda:client)
    monkeypatch.setattr(injection.time,'sleep',lambda _:None)


def test_socket_destruction_never_terminates_process(monkeypatch):
    client=Client([ROW,'']);use(monkeypatch,client)
    proof=injection.cut_owned_socket(ARGS)
    assert proof['method']=='socket-destroy' and not proof['after']
    assert not any(c[0] in ('ps','kill') for c in client.commands) and client.closed


def test_fallback_terminates_only_exact_revalidated_session(monkeypatch):
    client=Client([ROW,ROW,'']);use(monkeypatch,client)
    proof=injection.cut_owned_socket(ARGS)
    assert proof['method']=='exact-owned-session-termination' and not proof['after']
    assert [c for c in client.commands if c[0]=='kill']==[['kill','-TERM','49000']]
    assert client.closed


@pytest.mark.parametrize('changed_row,process',[(ROW.replace('49000','49001'),'sshd: root@notty'),(ROW,'/usr/sbin/sshd -D'),(ROW,'sshd: root [priv]')])
def test_changed_owner_or_master_process_cannot_be_terminated(monkeypatch,changed_row,process):
    client=Client([ROW,changed_row],process);use(monkeypatch,client)
    with pytest.raises(RuntimeError,match='refuse|Refuse'):
        injection.cut_owned_socket(ARGS)
    assert not any(c[0]=='kill' for c in client.commands) and client.closed
