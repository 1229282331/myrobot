"""Versioned nominal attachment model, shared by Servo and the read-only viewer."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).parent / 'assets' / 'workspace'


def manifest():
    return json.loads((ASSETS / 'manifest.json').read_text(encoding='utf-8'))


def verified_assets():
    value = manifest()
    content={k:v for k,v in value.items() if k!='model_id'}
    if hashlib.sha256(json.dumps(content,sort_keys=True).encode()).hexdigest()!=value['model_id']:
        raise ValueError('Workspace manifest checksum mismatch')
    for name, digest in value['files'].items():
        path = (ASSETS / name).resolve()
        if not path.is_relative_to(ASSETS.resolve()) or not path.is_file():
            raise ValueError('Missing workspace asset')
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('Workspace asset checksum mismatch: ' + name)
    return value


def attach(model, tcp_profile):
    if tcp_profile != 'omnipicker-r4':
        return model, None
    info = verified_assets()
    urdf = ET.fromstring(model['robot_description'])
    srdf = ET.fromstring(model['robot_description_semantic'])
    if urdf.find("link[@name='r4_plate']") is not None:
        raise ValueError('Attachment model already applied')
    # Sampling, display and Servo must use the same collision surfaces, not
    # separately generated approximations of the same source meshes.
    from copy import deepcopy
    assembled=ET.parse(ASSETS/'robot.urdf').getroot()
    for link in urdf.findall('link'):
        name=link.get('name')
        reference=assembled.find(f"link[@name='{name}']")
        if reference is None:raise ValueError('Control link absent from assembly: '+name)
        for kind in ('visual','collision'):
            for old in link.findall(kind):link.remove(old)
            for part in reference.findall(kind):
                part=deepcopy(part)
                for mesh in part.findall('.//mesh'):
                    mesh.set('filename',(ASSETS/mesh.get('filename')).resolve().as_uri())
                link.append(part)
    for joint in urdf.findall('joint'):
        reference=assembled.find(f"joint[@name='{joint.get('name')}']")
        if reference is None or ET.tostring(reference)!=ET.tostring(joint):
            # XML whitespace is not semantic; compare attributes recursively.
            def signature(e):return (e.tag,sorted(e.attrib.items()),[signature(c) for c in e])
            if reference is None or signature(reference)!=signature(joint):
                raise ValueError('Control kinematics differs from workspace assembly')
    additions = ET.parse(ASSETS / 'attachments.urdf').getroot()
    for mesh in additions.findall('.//mesh'):
        mesh.set('filename', (ASSETS / mesh.get('filename')).resolve().as_uri())
    for element in additions:
        urdf.append(element)
    for pair in info['allowed_contacts']:
        ET.SubElement(srdf, 'disable_collisions', link1=pair[0], link2=pair[1], reason='Attached mounting contact')
    if info.get('dynamic_gripper'):
        group=ET.SubElement(srdf,'group',name='e6_gripper_geometry')
        for name in [*info['dynamic_gripper']['movable_links'],info['dynamic_gripper']['fallback_link']]:
            ET.SubElement(group,'link',name=name)
    return dict(model, robot_description=ET.tostring(urdf, encoding='unicode'),
                robot_description_semantic=ET.tostring(srdf, encoding='unicode')), info['model_id']
