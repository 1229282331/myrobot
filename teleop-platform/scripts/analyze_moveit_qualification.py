"""Check virtual candidate direction with independent KDL FK readback."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def analyze(directory):
    summary=json.loads((directory/'summary.json').read_text())
    frames=[json.loads(line) for line in (directory/'fk_output.jsonl').read_text().splitlines()]
    if len(frames)!=len(summary['cases'])+1:raise ValueError('Incomplete independent FK readback')
    rows=[]
    for i,case in enumerate(summary['cases']):
        kind,axis,sign=case['name'].split('_');axis=int(axis);sign=int(sign)
        dp=np.array(frames[i+1]['position_m'])-frames[i]['position_m']
        dr=(Rotation.from_quat(frames[i+1]['quaternion_xyzw'])*
            Rotation.from_quat(frames[i]['quaternion_xyzw']).inv()).as_rotvec()
        delta=np.array(case['delta_q_rad']) if kind=='joint' else dp if axis<3 else dr
        index=axis if kind=='joint' else axis%3
        component=float(delta[index]*sign);norm=float(np.linalg.norm(delta))
        cosine=component/norm if norm else 0.
        rows.append(dict(case=case['name'],signed_component=component,direction_cosine=cosine,
            direction_passed=component>0 and cosine>.999,delta_position_m=dp.tolist(),
            delta_rotation_rad=dr.tolist(),statuses=case.get('statuses')))
    return dict(virtual_only=True,real_model_aligned=False,cases=rows,
        all_direction_checks_passed=all(r['direction_passed'] for r in rows),
        note='Direction only; not velocity, latency, physical collision or hardware acceptance')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=analyze(a.directory);a.output.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='cases'}))
