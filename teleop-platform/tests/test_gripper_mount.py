"""Installation regressions: shared frame, fixed TCP, and unchanged arm chain."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from e6_phosphobot.assembly_model import ASSETS, attach, verified_assets
from e6_phosphobot.tool_frame import get_tool


def test_confirmed_mount_b_matches_control_and_display():
    source=Path(__file__).resolve().parents[1]/'config/e6-controller-model'
    model=get_tool('omnipicker-r4').attach(dict(
        robot_description=(source/'e6.urdf').read_text(),
        robot_description_semantic=(source/'e6.srdf').read_text()))
    control,_=attach(model,'omnipicker-r4')
    for tree in (ET.fromstring(control['robot_description']),ET.parse(ASSETS/'robot.urdf').getroot()):
        mount=tree.find("joint[@name='omnipicker_mount']")
        assert mount.get('type')=='fixed'
        assert mount.find('parent').get('link')=='controller_flange'
        origin=mount.find('origin')
        np.testing.assert_allclose(list(map(float,origin.get('xyz').split())),[0,0,.008])
        r=Rotation.from_euler('xyz',list(map(float,origin.get('rpy').split())))
        np.testing.assert_allclose(r.apply([1,0,0]),[0,-1,0],atol=1e-14)
        np.testing.assert_allclose(r.apply([0,0,.1514]),[0,0,.1514],atol=1e-14)
        np.testing.assert_allclose(list(map(float,tree.find("joint[@name='gemini_e_mount']/origin").get('xyz').split())),[0,-.06,.02055])
    info=verified_assets()
    assert info['mounting']['rpy_rad']==[0,0,-np.pi/2]
    assert not any(any(x in ('Link2','Link3','Link4','Link5') for x in pair) for pair in info['allowed_contacts'])


def test_mount_config_is_the_regeneration_source():
    config=Path(__file__).resolve().parents[1]/'config/gripper_mount.json'
    assert json.loads(config.read_text(encoding='utf8'))==verified_assets()['mounting']
