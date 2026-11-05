"""Apply the confirmed rigid mounting transform to an existing assembly.

Keeps CAD, collision shapes, joint limits, TCP and contact exclusions intact.
Writes a new staging directory and invalidates the old reachability map.
No hardware IO. Also imported by the full CAD builder.
"""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'teleop-platform/config/gripper_mount.json'


def mounting():
    value = json.loads(CONFIG.read_text(encoding='utf-8'))
    assert value['parent'] == 'controller_flange' and value['child'] == 'omnipicker'
    for field in ('xyz_m', 'rpy_rad'):
        assert len(value[field]) == 3 and all(type(v) in (int, float) and math.isfinite(v) for v in value[field])
    return value


def apply(root, mount):
    joints = root.findall("joint[@name='omnipicker_mount']")
    assert len(joints) == 1
    joint = joints[0]
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == mount['parent']
    assert joint.find('child').get('link') == mount['child']
    origin = joint.find('origin')
    assert origin is not None
    origin.set('xyz', ' '.join(format(v, '.17g') for v in mount['xyz_m']))
    origin.set('rpy', ' '.join(format(v, '.17g') for v in mount['rpy_rad']))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage(source, target):
    source, target = source.resolve(), target.resolve()
    assert source != target and not target.exists()
    info = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    old_id = info.pop('model_id')
    assert hashlib.sha256(json.dumps(info, sort_keys=True).encode()).hexdigest() == old_id
    for name, digest in info['files'].items():
        path = (source/name).resolve()
        assert path.is_relative_to(source) and sha(path) == digest
    target.mkdir(parents=True)
    for name in info['files']:
        shutil.copy2(source/name, target/name)
    mount = mounting()
    for name in ('robot.urdf', 'attachments.urdf'):
        tree = ET.parse(target/name)
        apply(tree.getroot(), mount)
        tree.write(target/name, encoding='utf-8', xml_declaration=True)
    info['mounting'] = mount
    info['mount_revision_of'] = old_id
    for path in (CONFIG, Path(__file__)):
        info['sources'][path.relative_to(ROOT).as_posix()] = sha(path)
    info['files'] = {name: sha(target/name) for name in info['files']}
    info['model_id'] = hashlib.sha256(json.dumps(info, sort_keys=True).encode()).hexdigest()
    (target/'manifest.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
    print(json.dumps({'model_id':info['model_id'], 'previous':old_id, 'mounting':mount}))
    return info


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    stage(args.source, args.output)
