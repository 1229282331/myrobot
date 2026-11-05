"""Test explicit frame-definition hypotheses. Does NOT generate a control model."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def analyze(path):
    data=json.loads(path.read_text(encoding='utf-8'));samples=data['samples']
    model=Rotation.from_quat([s['model']['quaternion_xyzw'] for s in samples])
    control=Rotation.from_euler('xyz',[s['controller_pose_mm_deg'][3:] for s in samples],degrees=True)
    # Two discrete right-angle frame hypotheses motivated by the URDF joint1
    # origin and flange conventions; never estimate arbitrary fitted rotations.
    base=Rotation.from_euler('z',90,degrees=True)
    tip=Rotation.from_euler('z',180,degrees=True)
    error=np.rad2deg(((base*model*tip).inv()*control).magnitude())
    pm=np.array([s['model']['position_m'] for s in samples])*1000
    pc=np.array([s['controller_pose_mm_deg'][:3] for s in samples])
    residual=pc-base.apply(pm)
    return dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),samples=len(samples),
        sample_set=data.get('sample_set','local'),model_modified=False,production_eligible=False,
        hypothesis='Controller rotation = Rz(90 deg) * URDF rotation * Rz(180 deg)',
        orientation_error_deg_max=float(error.max()),
        position_residual_mm_min=residual.min(0).tolist(),position_residual_mm_max=residual.max(0).tolist(),
        position_error_mm_max=float(np.linalg.norm(residual,axis=1).max()),
        note='Frame hypothesis only; unresolved translation/geometry. No fitted constant offset applied.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('summary',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=analyze(a.summary);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))
