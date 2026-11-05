"""Publish static UI only after the browser control owner has been released."""
import argparse,hashlib,json,os,shutil,time,urllib.request
from pathlib import Path
root=Path(__file__).resolve().parents[2];platform=root/'teleop-platform'
parser=argparse.ArgumentParser();parser.add_argument('--rollback',action='store_true');parser.add_argument('--check',action='store_true');args=parser.parse_args()
source=root/'output/optimization-v274/baseline/dist' if args.rollback else platform/'upstream/phosphobot/dashboard/dist'
destination=platform/'upstream/phosphobot/phosphobot/resources/dist'
evidence=root/'docs/teleop-rebuild/evidence/optimization-v274'
def hashes(base):return {p.relative_to(base).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(base.rglob('*')) if p.is_file()}
states=[]
for _ in range(10):
    with urllib.request.urlopen('http://127.0.0.1:8765/e6/state',timeout=3) as r:s=json.load(r)
    assert not s['ready'] and s['stationary'] and s['recording']['phase'] in ('idle','complete','aborted'), 'Not idle'
    assert s['feedback']['connected'] and s['feedback']['age_ms']<=100, 'Feedback not fresh'
    states.append(s);time.sleep(.1)
if args.rollback:
    manifest=json.loads((source.parent/'sha256.json').read_text())
    assert all(hashlib.sha256((source.parent/name).read_bytes()).hexdigest()==sha for name,sha in manifest.items())
expected=hashes(source);assert 'index.html' in expected
evidence.mkdir(parents=True,exist_ok=True)
(evidence/('rollback-preflight.json' if args.rollback else 'publish-preflight.json')).write_text(json.dumps(states,indent=2),encoding='utf8')
if not args.check:
    # Content-addressed assets first; change the entry point only when complete.
    for name in expected:
        if name=='index.html':continue
        target=destination/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/name,target)
    temp=destination/'index-v274.tmp';shutil.copyfile(source/'index.html',temp);os.replace(temp,destination/'index.html')
    assert all(hashlib.sha256((destination/name).read_bytes()).hexdigest()==sha for name,sha in expected.items())
    if not args.rollback:
        (destination/'e6-build.json').write_text(json.dumps({'extensions':hashes(platform/'frontend'),'artifacts':expected},indent=2),encoding='utf8')
    (evidence/('rollback.json' if args.rollback else 'published-build.json')).write_text(json.dumps({'artifacts':expected,'extensions':hashes(platform/'frontend'),'arm_command_id':s['feedback']['status']['current_command_id']},indent=2),encoding='utf8')
print(json.dumps({'checked':True,'published':not args.check,'rollback':args.rollback,'files':len(expected),'command_id':s['feedback']['status']['current_command_id']}))
