"""Session-fixed control point; controller Tool0 stays at the flange.

The OmniPicker profile is a nominal CAD datum, not aperture telemetry or a
physical TCP calibration. See TCP_QUALIFICATION.md for geometry provenance.
"""
from dataclasses import dataclass
import hashlib
import json
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class ToolFrame:
    name: str
    frame: str
    label: str
    translation_m: tuple
    quaternion_xyzw: tuple = (0., 0., 0., 1.)
    calibration: str = 'controller_datum'

    def apply(self, position, rotation):
        import numpy as np
        from scipy.spatial.transform import Rotation
        return (np.asarray(position) + rotation.apply(self.translation_m),
                rotation * Rotation.from_quat(self.quaternion_xyzw))

    def pose(self, position, rotation):
        p, r = self.apply(position, rotation)
        return dict(position_m=p.tolist(), quaternion_xyzw=r.as_quat().tolist(),
                    pose_frame=self.frame)

    def description(self):
        value = dict(profile=self.name, frame=self.frame, label=self.label,
                     parent_frame='controller_flange', translation_m=list(self.translation_m),
                     quaternion_xyzw=list(self.quaternion_xyzw), calibration=self.calibration,
                     opening_policy='fixed_reference_not_aperture_tracking')
        value['sha256'] = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
        return value

    def attach(self, model):
        """Extend a verified model in memory; never rewrite the pinned artifacts."""
        from scipy.spatial.transform import Rotation
        if self.name == 'flange':
            return model
        urdf = ET.fromstring(model['robot_description'])
        if urdf.find(f"link[@name='{self.frame}']") is not None:
            raise ValueError('TCP already attached; refuse a double offset')
        if urdf.find("link[@name='controller_flange']") is None:
            raise ValueError('Missing controller flange datum')
        ET.SubElement(urdf, 'link', name=self.frame)
        joint = ET.SubElement(urdf, 'joint', name=self.frame+'_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link='controller_flange')
        ET.SubElement(joint, 'child', link=self.frame)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, self.translation_m)),
                      rpy=' '.join(map(str, Rotation.from_quat(self.quaternion_xyzw).as_euler('xyz'))))
        srdf = ET.fromstring(model['robot_description_semantic'])
        chain = srdf.find("group[@name='me6_group']/chain")
        if chain is None or chain.get('tip_link') != 'controller_flange':
            raise ValueError('Unexpected control chain tip')
        chain.set('tip_link', self.frame)
        return dict(model, robot_description=ET.tostring(urdf, encoding='unicode'),
                    robot_description_semantic=ET.tostring(srdf, encoding='unicode'))


def get_tool(name='flange'):
    profiles = {
        'flange': ToolFrame('flange', 'controller_flange', '法兰原点', (0., 0., 0.)),
        # Mean height of opposing distal inner CAD pads (143.35 mm) + R2
        # mounting thickness (7 mm), rounded to 0.1 mm. Lateral symmetry
        # datum, axes parallel to Tool0; no unmeasured mounting yaw claimed.
        'omnipicker-r2': ToolFrame('omnipicker-r2', 'grasp_tcp', '夹指抓握中心',
                                  (0., 0., .1504), calibration='nominal_cad_unmeasured'),
        # Same nominal pad datum with the supplied PD-01 R4 (8 mm) plate.
        # Preserve R2 for old sessions; tool axes remain parallel to Tool0.
        'omnipicker-r4': ToolFrame('omnipicker-r4', 'grasp_tcp', '夹指抓握中心',
                                  (0., 0., .1514), calibration='nominal_cad_unmeasured'),
    }
    if name not in profiles:
        raise ValueError('Unknown TCP profile')
    return profiles[name]
