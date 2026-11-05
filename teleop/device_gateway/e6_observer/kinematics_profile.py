"""Recompute a local calibration from immutable, read-only controller capture.

Both hosts verify this bundle. A profile alone never unlocks live/operator IO.
"""
import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .control_math import angle, pose
from .local_kinematics import LocalKinematics
from .protocol import FeedbackParser, SDK_COMMIT
from .control_core import BOUNDS


def source_sha():
    return {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in ('local_kinematics.py', 'kinematics_profile.py', 'control_math.py')}


def load_profile(path, robot_ip='192.168.1.4'):
    path = Path(path)
    p = json.loads(path.read_text(encoding='utf-8'))
    if (p.get('schema_version') != 1 or p.get('dataset_kind') != 'controller_kinematics_calibration'
            or p.get('robot_ip') != robot_ip or p.get('user') != 0 or p.get('tool') != 0
            or p.get('layout_source_commit') != SDK_COMMIT or p.get('source_sha256') != source_sha()):
        raise ValueError('Local calibration identity/source/configuration mismatch')
    name = p.get('capture_file')
    if not isinstance(name, str) or Path(name).name != name or '/' in name or '\\' in name:
        raise ValueError('Calibration capture must be a sibling file')
    raw = (path.parent/name).read_bytes()
    if hashlib.sha256(raw).hexdigest() != p.get('capture_sha256'):
        raise ValueError('Calibration capture checksum mismatch')
    if name.endswith('.gz'):
        with gzip.open(path.parent/name, 'rb') as stream:
            raw = stream.read(64*1024*1024+1)
    if len(raw) > 64*1024*1024:
        raise ValueError('Calibration capture too large')
    capture = json.loads(raw)
    if (capture.get('failure') or capture.get('robot_ip') != robot_ip
            or any(capture.get(k, 1) != 0 for k in ('motion_tx', 'stop_tx', 'can_tx'))):
        raise ValueError('Calibration requires a complete read-only capture')
    anchor = capture['first_feedback']['decoded']['joints_rad']
    rows = capture['rows']; fit = []; held = []
    seen = set()
    for row in rows:
        state = row['input_feedback']; d = state['decoded']
        parsed = FeedbackParser(extended=True).feed(base64.b64decode(state['raw_base64'], validate=True))
        same = len(parsed) == 1 and set(parsed[0][1]) == set(d)
        if same:
            for key,value in parsed[0][1].items():
                expected=d[key]
                if isinstance(value,list):
                    same = same and np.asarray(value).shape == np.asarray(expected).shape and np.allclose(value,expected,atol=1e-12,rtol=0)
                else:same = same and value == expected
        if not same:
            raise ValueError('Calibration raw/decoded feedback mismatch')
        if (d['layout_source_commit'] != SDK_COMMIT or d['robot_mode'] not in (4, 5)
                or d['enable_status'] != int(d['robot_mode'] == 5) or d['error_status'] or d['collision_state']
                or d.get('run_queued_command') or d['user_index'] or d['tool_index']
                or any(abs(v) > 1e-8 for v in d['user_configuration_raw_mm_deg']+d['tool_configuration_raw_mm_deg'])
                or np.max(np.abs(np.asarray(d['joints_rad'])-anchor)) > math.radians(.1)):
            raise ValueError('Calibration device moved or configuration changed')
        q = tuple(row['joints_rad'])
        if np.asarray(q).shape != (6,) or not np.isfinite(q).all() or np.any(np.abs(q)>BOUNDS-math.radians(5)):
            raise ValueError('Calibration virtual joints exceed limits')
        # Independent validation must not accidentally reuse fitting points.
        if q in seen:
            raise ValueError('Calibration duplicated fitting/holdout points')
        seen.add(q)
        split = row['request']['split']
        if split not in ('fit', 'holdout'):
            raise ValueError('Unknown calibration split')
        (fit if split == 'fit' else held).append(row)
        command = row['events'][-1]
        expected = 'PositiveKin('+','.join(f'{v:.6f}' for v in np.degrees(q))+',user=0,tool=0)'
        if command.get('command') != expected or not command.get('acknowledged'):
            raise ValueError('Calibration FK acknowledgement does not match target')
        reply = command.get('raw_reply', '')
        if reply != '0,{'+','.join(str(v) for v in row['pose_mm_deg'])+'},'+expected+';':
            # Controller formats decimals independently. Parse values, retaining
            # the complete raw response/command in the source capture.
            import re
            match = re.fullmatch(r'\s*0\s*,\s*\{(.*?)\}\s*,\s*(.*?)\s*;', reply)
            if not match or match[2].replace(' ', '') != expected or [float(v) for v in match[1].split(',')] != row['pose_mm_deg']:
                raise ValueError('Calibration raw FK response mismatch')
    if len(held) < 300:
        raise ValueError('Calibration needs independent interior/boundary validation')
    span=math.radians(10)
    held_delta=np.abs(np.asarray([v['joints_rad'] for v in held])-anchor)
    if np.sum(np.all(np.isclose(held_delta,span,atol=1e-10,rtol=0),axis=1))!=64 or np.sum(np.all(held_delta<span-1e-10,axis=1))<256:
        raise ValueError('Calibration validation lacks all domain corners/interior samples')
    model = LocalKinematics.fit([v['joints_rad'] for v in fit], [v['pose_mm_deg'] for v in fit],
                                anchor, capture['anchor_pose_mm_deg'], math.radians(10))
    max_position = max_rotation = 0.
    for row in held:
        p0, r0 = model.fk(row['joints_rad']); pp, rr = pose(row['pose_mm_deg'])
        max_position = max(max_position, float(np.linalg.norm(p0-pp)))
        max_rotation = max(max_rotation, angle(r0, rr))
    if max_position > .001 or max_rotation > math.radians(1):
        raise ValueError('Calibration independent holdout exceeds 1 mm / 1 degree')
    metadata = dict(kind='controller-local', profile_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    capture_sha256=p['capture_sha256'], model_domain_joint_span_rad=model.span,
                    fit_samples=len(fit), holdout_samples=len(held),
                    holdout_max_position_m=max_position, holdout_max_rotation_rad=max_rotation,
                    physical_accuracy_verified=False, anchor_joints_rad=model.anchor_q.tolist())
    return model, metadata
