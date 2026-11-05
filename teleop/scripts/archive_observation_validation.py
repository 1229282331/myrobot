"""Archive final software checks and hashes, without contacting any device."""
from datetime import datetime,timezone
import hashlib
import argparse
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser();parser.add_argument('--evidence',type=Path,default=root/'docs/evidence/teleop_observation_20261004');args=parser.parse_args()
evidence=args.evidence.resolve()
commands=[([sys.executable,'-X','utf8','-m','pytest','teleop/tests','-q'],root),
          (['npm.cmd','test'],root/'teleop/web'),(['npm.cmd','run','build'],root/'teleop/web')]
results=[]
for command,cwd in commands:
    run=subprocess.run(command,cwd=cwd,capture_output=True,encoding='utf-8',errors='replace')
    results.append(dict(command=command,exit_code=run.returncode,output=run.stdout+run.stderr))
    if run.returncode:raise RuntimeError(results[-1]['output'])
versions={name:importlib.metadata.version(name) for name in ('mujoco','lerobot','torch','av','websockets','paramiko','zstandard')}
report=dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),python=sys.version,versions=versions,results=results,
            official_lerobot_readback=json.loads((root/'datasets/e6_teleop/lerobot/acceptance_final/simulation_validation.json').read_text(encoding='utf-8')))
evidence.mkdir(parents=True,exist_ok=True)
(evidence/'software_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
files=[]
for directory in ('teleop/src','teleop/device_gateway','teleop/scripts','teleop/tests','teleop/web/src'):
    files.extend(p for p in (root/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
files.extend(root/name for name in ('teleop/pyproject.toml','teleop/uv.lock','teleop/launch-observe.cmd','teleop/web/package.json','teleop/web/package-lock.json'))
manifest=[dict(path=p.relative_to(root).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(files)]
(evidence/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(all_checks_passed=True,files_hashed=len(manifest),versions=versions),ensure_ascii=False))
