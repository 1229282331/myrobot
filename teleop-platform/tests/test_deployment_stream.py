import hashlib
import pytest
from e6_phosphobot import launcher


@pytest.mark.parametrize('failed_close', [False, True])
def test_pipeline_drained_before_release_manifest(tmp_path, monkeypatch, failed_close):
    payload={'e6_phosphobot/test.py':b'x'*100000}
    manifest={k:hashlib.sha256(v).hexdigest() for k,v in payload.items()}
    monkeypatch.setattr(launcher,'make_payload',lambda:(payload,manifest))
    monkeypatch.setattr(launcher,'remote',lambda *a:None)
    events=[]
    class File:
        def __init__(self,name):self.name=name
        def __enter__(self):events.append(('open',self.name));return self
        def set_pipelined(self,value):assert value;events.append(('pipeline',self.name))
        def write(self,data):events.append(('write',self.name))
        def __exit__(self,*a):
            events.append(('drained',self.name))
            if failed_close and self.name.endswith('test.py'):raise OSError('write ack failed')
    class Sftp:
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def mkdir(self,*a):pass
        def open(self,name,mode):return File(name)
    class Client:
        def open_sftp(self):return Sftp()
    if failed_close:
        with pytest.raises(OSError):launcher.deploy(Client(),{'remote_base':'/test'},'session',tmp_path)
        assert not any(name.endswith('session.json') for _,name in events)
        assert not (tmp_path/'deployment.json').exists()
    else:
        launcher.deploy(Client(),{'remote_base':'/test'},'session',tmp_path)
        actions=[kind for kind,name in events]
        assert actions==['open','pipeline','write','drained','open','write','drained']
        assert (tmp_path/'deployment.json').exists()
