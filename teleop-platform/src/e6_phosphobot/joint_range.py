"""Absolute bounds from the reviewed E6 URDF, shared with MoveIt Servo."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

JOINT_MARGIN_DEG = 5.


class JointRange:
    def __init__(self, bundle):
        model = Path(bundle) / 'model'
        profile = json.loads((model / 'profile.json').read_text())
        data = (model / 'e6.urdf').read_bytes()
        self.sha = hashlib.sha256(data).hexdigest()
        if self.sha != profile['urdf_sha256']:
            raise ValueError('Joint range model differs from its reviewed profile')
        tree = ET.fromstring(data)
        bounds = []
        for i in range(1, 7):
            joint = tree.find(f"joint[@name='joint{i}']")
            if joint is None or joint.get('type') != 'revolute':
                raise ValueError('Expected bounded E6 revolute joint')
            limit = joint.find('limit')
            bounds.append([float(limit.get('lower')), float(limit.get('upper'))])
        self.bounds = np.array(bounds)
        self.lower = self.bounds[:, 0] + math.radians(JOINT_MARGIN_DEG)
        self.upper = self.bounds[:, 1] - math.radians(JOINT_MARGIN_DEG)
        if not np.all(np.isfinite(self.bounds)) or np.any(self.lower >= self.upper):
            raise ValueError('Invalid model joint range')

    def description(self):
        return dict(source='reviewed_e6_urdf', urdf_sha256=self.sha,
                    margin_deg=JOINT_MARGIN_DEG,
                    lower_deg=np.rad2deg(self.lower).tolist(),
                    upper_deg=np.rad2deg(self.upper).tolist())

    def evaluate(self, actual, candidate):
        actual, candidate = np.asarray(actual), np.asarray(candidate)
        if actual.shape != (6,) or candidate.shape != (6,) or not np.all(np.isfinite([actual, candidate])):
            raise ValueError('Invalid joint range input')
        def excess(q):
            return np.maximum(self.lower-q, 0) + np.maximum(q-self.upper, 0)
        before, after = excess(actual), excess(candidate)
        retreat = bool(np.any(before > 0) and np.all(after <= before) and np.any(after < before))
        return dict(allowed=bool(np.all(after == 0) or retreat), retreat=retreat,
                    exceeded=[f'J{i+1}' for i, value in enumerate(after) if value > 0],
                    actual_excess_rad=before.tolist(), candidate_excess_rad=after.tolist())
