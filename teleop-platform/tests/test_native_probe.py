from copy import deepcopy
import json
from pathlib import Path
import queue
from types import SimpleNamespace

import pytest

from e6_phosphobot import native_probe as probe


@pytest.mark.parametrize('mode,delta',[('zero-joints',0),('stop-j4',.32),('stop-tool-rx',.5)])
@pytest.mark.parametrize('duration',[.05,.1])
def test_official_joint_probe_uses_fresh_seed_and_always_stops(monkeypatch,tmp_path,mode,delta,duration):
    fixture=Path(__file__).resolve().parents[2]/'docs/teleop-rebuild/evidence/native-gain750-zero-v14/summary.json'
    current=json.loads(fixture.read_text())['before'];seed=current['status']['joint_state']['q_actual']
    now=[100.];sent=[];sent_at=[];camera_checks=[]
    monkeypatch.setattr(probe,'time',SimpleNamespace(monotonic=lambda:now[0],
        perf_counter_ns=lambda:int(now[0]*1e9),sleep=lambda seconds:now.__setitem__(0,now[0]+max(.001,seconds))))
    class Reader:
        events=[]
        def __init__(self,*a):pass
        def start(self):pass
        def close(self):pass
        def subscribe(self,*a):return SimpleNamespace(queue=queue.Queue(),dropped=0)
        def snapshot(self):
            data=deepcopy(current);data.update(received_ns=int(now[0]*1e9),age_ms=0)
            return data
    class Connection:
        def __init__(self,*a):self.events=[]
        def connect(self):pass
        def disconnect(self):pass
        def set_timeout(self,**kw):pass
        def send_receive_text(self,text):
            sent.append(text);sent_at.append(now[0]);return '0,{},'+text+';'
    monkeypatch.setattr(probe,'FeedbackReader',Reader)
    monkeypatch.setattr(probe,'FramedDobotConnection',Connection)
    monkeypatch.setattr(probe,'checked_camera',lambda p:camera_checks.append(p))
    monkeypatch.setattr(probe,'__name__','e6_phosphobot.native_probe')
    monkeypatch.setattr('sys.argv',['probe','--mode',mode,'--servo-gain','750',
        '--servo-duration-s',str(duration),'--output',str(tmp_path/'probe')])
    assert probe.main()==0
    motion=[x for x in sent if x.startswith('Servo')]
    assert len(motion)==(10 if mode=='stop-tool-rx' else 20)
    assert all(x.startswith('ServoP(' if mode=='stop-tool-rx' else 'ServoJ(') for x in motion)
    assert all(f't={duration:.3f}' in x for x in motion)
    times=[t for text,t in zip(sent,sent_at) if text.startswith('Servo')]
    assert [b-a for a,b in zip(times,times[1:])]==pytest.approx([.05]*(len(motion)-1))
    target=[float(x) for x in motion[-1][7:].split(',')[:6]]
    if mode=='stop-tool-rx':
        seed=[current['status']['tool_vector_actual'][k] for k in ('x','y','z','rx','ry','rz')]
        assert sent_at[sent.index('Stop()')]==pytest.approx(times[-1])
    for i,value in enumerate(seed):assert target[i]==pytest.approx(value+(delta if i==3 else 0),abs=1e-6)
    assert sent[-1]=='Stop()' and len(camera_checks)>20
    assert not any(x.startswith(('EnableRobot','ClearError','Continue','Acc','SpeedFactor')) for x in sent)


def test_native_line_plan_is_from_given_fresh_pose_and_rejects_ik_jumps():
    import numpy as np
    class Kinematics:
        def __init__(self,jump=False):self.pose=None;self.jump=jump
        def InverseKin(self,pose,**kw):
            self.pose=pose
            assert kw['use_joint_near']==1 and kw['user']==kw['tool']==0
            joints=np.array(kw['joint_near']);joints[3]+=10 if self.jump else (pose[2]-410)/5
            return '0,{'+','.join(map(str,joints))+'},InverseKin(...);'
        def PositiveKin(self,joints,**kw):return '0,{'+','.join(map(str,self.pose))+'},PositiveKin(...);'
    seed=np.array([-71.,-58.,102.,-89.,98.,12.])
    plan=probe.joint_z_plan(Kinematics(),[-13.,-184.,410.,-136.,14.,-164.],seed,lambda:None)
    assert plan[-1]['pose_mm_deg'][2]==pytest.approx(411.6)
    assert plan[-1]['joints_deg'][3]==pytest.approx(seed[3]+.32)
    x_plan=probe.joint_z_plan(Kinematics(),[-13.,-184.,410.,-136.,14.,-164.],seed,lambda:None,axis=0)
    assert x_plan[-1]['pose_mm_deg']==pytest.approx([-11.4,-184.,410.,-136.,14.,-164.])
    with pytest.raises(RuntimeError,match='rejected'):
        probe.joint_z_plan(Kinematics(True),[-13.,-184.,410.,-136.,14.,-164.],seed,lambda:None)


def test_micro_probe_preserves_position_and_base_rotation_direction():
    import numpy as np
    from scipy.spatial.transform import Rotation
    pose=[-22.,-203.,438.,-150.,20.,177.]
    plan=probe.micro_rotation_plan(pose)
    assert len(plan)==40
    assert np.array(plan)[:,:3]==pytest.approx(np.tile(pose[:3],(40,1)))
    initial=Rotation.from_euler('xyz',pose[3:],degrees=True)
    delta=(Rotation.from_euler('xyz',np.array(plan)[:,3:],degrees=True)*initial.inv()).as_rotvec(degrees=True)
    assert delta==pytest.approx(np.c_[np.zeros(40),-np.arange(1,41)*.0125,np.zeros(40)],abs=1e-10)
    with pytest.raises(ValueError):probe.micro_rotation_plan(pose,speed_deg_s=10)
