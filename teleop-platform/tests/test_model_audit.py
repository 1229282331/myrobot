from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from e6_phosphobot import model_audit


@pytest.mark.parametrize('tool_offset',[0,1])
def test_model_audit_is_read_only_and_rejects_nonzero_tool(monkeypatch,tmp_path,tool_offset):
    source=Path(__file__).resolve().parents[2]/'docs/teleop-rebuild/evidence/native-gain750-ik-z-v16/summary.json'
    snap=json.loads(source.read_text(encoding='utf-8'))['before'];snap['age_ms']=0
    snap['status']['tool_value'][0]=tool_offset
    commands=[];closed=[]
    class Reader:
        events=[]
        def __init__(self,*a):pass
        def start(self):pass
        def close(self):closed.append('feedback')
        def snapshot(self):return deepcopy(snap)
    class Connection:
        events=[]
        def __init__(self,*a):pass
        def connect(self):commands.append('connect')
        def disconnect(self):closed.append('dashboard')
        def send_receive_text(self,text):
            assert text.startswith('PositiveKin(')
            commands.append(text);return '0,{0,0,0,0,0,0},'+text+';'
    monkeypatch.setattr(model_audit,'FeedbackReader',Reader)
    monkeypatch.setattr(model_audit,'FramedDobotConnection',Connection)
    def kdl(*a,**kw):
        rows=kw['input'].strip().splitlines()
        assert len(rows)==49 and all(len(r.split())==6 for r in rows)
        return SimpleNamespace(stdout='\n'.join(json.dumps(dict(position_m=[.01,0,0],quaternion_xyzw=[0,0,0,1])) for r in rows))
    monkeypatch.setattr(model_audit.subprocess,'run',kdl)
    urdf=tmp_path/'test.urdf';urdf.write_text('<robot/>');out=tmp_path/'out'
    monkeypatch.setattr('sys.argv',['audit','--urdf',str(urdf),'--probe','unused','--output',str(out)])
    assert model_audit.main()==bool(tool_offset)
    result=json.loads((out/'summary.json').read_text())
    assert result['motion_commands']==result['can_sends']==result['parameter_writes']==0
    assert set(closed)=={'feedback','dashboard'}
    if tool_offset:assert commands==[] and result['error']
    else:
        assert len(commands)==50 and result['max_position_error_mm']==pytest.approx(10)
        assert not result['aligned_under_1mm_1deg']
