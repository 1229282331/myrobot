"""Per-epoch CAD sweeps of recorded shadow paths, without device connections.

The reduced inputs retain unchanged base/poses/joints and exact original IDs.
Full input/provenance remains in the source artifact. No new trajectory is made.
"""
import argparse
from contextlib import redirect_stdout
import gzip
import hashlib
import io
import json
from pathlib import Path

from analyze_workspace_sweep import analyze


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    raw=a.source.read_bytes();decoded=gzip.decompress(raw) if a.source.suffix=='.gz' else raw
    data=json.loads(decoded);assert data['last']['stage']=='shadow'
    groups={}
    for c in data['candidates']:groups.setdefault(c['moveit']['epoch'],[]).append(c)
    results=[]
    for epoch,candidates in groups.items():
        intent=candidates[0]['moveit']['source_input']['intent']
        reduced=dict(base=data['base'],last={'stage':'shadow'},counts=data['counts'],
            source_compressed_sha256=hashlib.sha256(raw).hexdigest(),
            source_uncompressed_sha256=hashlib.sha256(decoded).hexdigest(),
            source_epoch=epoch,source_ids=[c['moveit']['id'] for c in candidates],
            candidates=[{k:c[k] for k in ('position_m','quaternion_xyzw','joints_rad')} for c in candidates])
        source=a.output/f'epoch-{epoch}.json';source.write_text(json.dumps(reduced),encoding='utf8')
        output=a.output/f'epoch-{epoch}-sweep.json'
        with redirect_stdout(io.StringIO()):analyze(source,output)
        result=json.loads(output.read_text())
        results.append(dict(epoch=epoch,intent=intent,candidates=len(candidates),
            maximum_body_displacement_mm=max(result['per_link_corner_displacement_mm'].values()),
            flange_displacement_mm=result['flange_displacement_mm'],
            flange_rotation_deg=result['flange_rotation_deg'],
            assumed_attachment_displacement_mm=result['attachment_displacement_bound_mm'],
            joint_min_deg=result['joint_min_deg'],joint_max_deg=result['joint_max_deg'],
            projection=source.name,sweep=output.name))
    (a.output/'summary.json').write_text(json.dumps(dict(source=str(a.source),
        source_sha256=hashlib.sha256(raw).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        note='Per-epoch bounds use the same original actual baseline. The 0.30m attachment radius is assumed, not physically measured. No hardware motion evidence.',
        groups=results),indent=2),encoding='utf8')
    print(json.dumps(results))


if __name__=='__main__':main()
