"""Publish/rollback v279 while the owned session is released. No device IO."""
from pathlib import Path
import argparse, hashlib, json, os, shutil, socket

root = Path(__file__).resolve().parents[2]
platform = root/'teleop-platform'
bundle = root/'output/operation-regression-v279'
p = argparse.ArgumentParser()
p.add_argument('--rollback', action='store_true')
p.add_argument('--check', action='store_true')
a = p.parse_args()
package = bundle/('baseline' if a.rollback else 'candidate')
manifest = json.loads((package/'sha256.json').read_text())
assert all(hashlib.sha256((package/name).read_bytes()).hexdigest() == sha for name,sha in manifest.items())
for port in (8765,8776,8777,8889):
    with socket.socket() as sock:
        sock.settimeout(.2)
        assert sock.connect_ex(('127.0.0.1',port)) != 0, 'Release owned session normally first'
if not a.check:
    for folder, dest in [('backend',platform/'src/e6_phosphobot'),('frontend',platform/'frontend')]:
        for source in (package/folder).glob('*'):
            shutil.copy2(source,dest/source.name)
    shutil.copy2(package/'workstation.json', platform/'config/workstation.json')
    dest = platform/'upstream/phosphobot/phosphobot/resources/dist'
    for source in (package/'dist').rglob('*'):
        if not source.is_file() or source.name == 'index.html': continue
        target = dest/source.relative_to(package/'dist')
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
    temp = dest/'index-v279.tmp'
    shutil.copy2(package/'dist/index.html',temp)
    os.replace(temp,dest/'index.html')
    (root/'evidence/operation-regression-v279'/('rollback.json' if a.rollback else 'published.json')).write_text(json.dumps(manifest,indent=2))
print(json.dumps(dict(checked=True,published=not a.check,rollback=a.rollback,files=len(manifest))))
