from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from e6_phosphobot.tool_frame import get_tool
from e6_phosphobot.control_state import for_operator
from e6_phosphobot.gateway import NativeExecutor
from e6_phosphobot.session_runner import motion_engine_option
from test_gateway import setup
from test_moveit_bridge import Bridge, row


@pytest.mark.parametrize('profile,offset',[('omnipicker-r2',.1504),('omnipicker-r4',.1514)])
def test_flange_compensation_keeps_selected_point_across_rotation_wrap(profile,offset):
    tool=get_tool(profile)
    desired=np.array([.2,-.1,.3])
    for euler in ([0,0,0],[0,90,0],[179.9,-30,-179.9],[-179.9,-30,179.9]):
        rotation=Rotation.from_euler('xyz',euler,degrees=True)
        flange=desired-rotation.apply([0,0,offset])
        point,orientation=tool.apply(flange,rotation)
        np.testing.assert_allclose(point,desired,atol=1e-12)
        assert (orientation.inv()*rotation).magnitude()<1e-12
    # A flange-only rotation demonstrably moves the desired grip point.
    before,_=tool.apply([0,0,0],Rotation.identity())
    after,_=tool.apply([0,0,0],Rotation.from_euler('y',90,degrees=True))
    np.testing.assert_allclose(after-before,[offset,0,-offset],atol=1e-12)


@pytest.mark.parametrize('profile,offset',[('omnipicker-r2',.1504),('omnipicker-r4',.1514)])
def test_urdf_chain_ends_at_tcp_and_original_model_remains_verifiable(profile,offset):
    root=Path(__file__).resolve().parents[1]/'config/e6-controller-model'
    original=dict(robot_description=(root/'e6.urdf').read_text(),
                  robot_description_semantic=(root/'e6.srdf').read_text())
    saved=deepcopy(original);tool=get_tool(profile);model=tool.attach(original)
    assert original==saved
    chain=ET.fromstring(model['robot_description_semantic']).find("group[@name='me6_group']/chain")
    assert chain.get('tip_link')=='grasp_tcp'
    joint=ET.fromstring(model['robot_description']).find("joint[@name='grasp_tcp_fixed']")
    assert joint.find('parent').get('link')=='controller_flange'
    assert joint.find('origin').get('xyz')==f'0.0 0.0 {offset}'
    with pytest.raises(ValueError,match='double offset'):tool.attach(model)


@pytest.mark.parametrize('profile',['omnipicker-r2','omnipicker-r4'])
def test_no_silent_tool_change_on_native_route(profile):
    with pytest.raises(ValueError,match='requires MoveIt'):
        NativeExecutor('test','shadow',None,tcp_profile=profile)
    with pytest.raises(ValueError,match='requires MoveIt'):
        motion_engine_option(dict(motion_engine='native_servop',tcp_profile=profile))
    assert '--tcp-profile' in motion_engine_option(dict(rootfs='/r',tcp_profile=profile))


def test_launcher_profile_validation_requires_no_numerical_packages():
    source=str(Path(__file__).resolve().parents[1]/'src')
    code=(f'import sys; sys.path.insert(0,{source!r}); '
          'from e6_phosphobot.session_runner import motion_engine_option; '
          "assert '--tcp-profile' in motion_engine_option(dict(rootfs='/r',tcp_profile='omnipicker-r2'))")
    subprocess.run([sys.executable,'-S','-c',code],check=True,capture_output=True)


@pytest.mark.parametrize('profile,offset',[('omnipicker-r2',.1504),('omnipicker-r4',.1514)])
def test_gateway_sends_joint_target_and_records_tcp_plus_original_flange(profile,offset):
    e,d,send,step,now=setup(moveit_bundle='fixture',tcp_profile=profile)
    bridge=Bridge();bridge.info['tcp']=e.tool.description();e.moveit=bridge
    original=e.base[0].copy()
    np.testing.assert_allclose(original,[0,0,.4+offset])
    # Independently make a small rotation about the fixed grip point.
    rotation=Rotation.from_euler('x',.25,degrees=True)
    flange=original-rotation.apply([0,0,offset])
    send(deadman=True,angular_rad_s=[.01,0,0])
    candidate=row(now[0],e.session.sequence)
    candidate.update(position_m=flange.tolist(),quaternion_xyzw=rotation.as_quat().tolist())
    bridge.row=candidate;step()
    assert d.calls[0][0]=='ServoJ'
    np.testing.assert_allclose(d.calls[0][1][0],np.rad2deg(candidate['joints_rad']))
    np.testing.assert_allclose(e.candidate['position_m'],original,atol=1e-12)
    np.testing.assert_allclose(e.candidate['flange_pose']['position_m'],flange)
    assert e.candidate['pose_frame']=='grasp_tcp'
    assert e.candidate['tcp']==e.state()['capabilities']['tcp']
    compact=for_operator(e.state())
    assert compact['candidate']['flange_pose']==e.candidate['flange_pose']
    np.testing.assert_allclose(compact['actual']['position_m'],original)
    np.testing.assert_allclose(compact['actual']['flange_pose']['position_m'],[0,0,.4])
    send();step()
    assert e.session.ready and d.calls[-1][0]=='Stop'
    assert e.base[0].tolist()==original.tolist()


def test_controller_flange_feedback_health_is_not_redefined_by_tcp():
    e,d,send,step,now=setup(moveit_bundle='fixture',tcp_profile='omnipicker-r2')
    assert e.healthy(d.snapshot()) is None
    d.data['status']['tool_value'][2]=150.4
    assert e.healthy(d.snapshot()) is not None  # No second offset in Tool0.


def test_old_model_cannot_accept_grip_point_input():
    e,d,send,step,now=setup(moveit_bundle='fixture',tcp_profile='omnipicker-r2')
    e.moveit=Bridge()  # No TCP attestation from legacy solver.
    send(deadman=True,angular_rad_s=[.01,0,0]);step()
    assert not any(call[0]=='ServoJ' for call in d.calls)
    assert not e.session.ready
