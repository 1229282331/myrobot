"""Apply reviewed patches and copy our explicit E6 extension into the pinned app."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import os
import tempfile


def matches_complete_patch_chain(upstream, patches):
    """Later UI patches may replace context from earlier ones.

    Compare against the complete chain in an isolated Git index, without
    resetting the checkout or accepting unreviewed local edits.
    """
    with tempfile.TemporaryDirectory(prefix='e6-ui-index-') as temporary:
        env = dict(os.environ, GIT_INDEX_FILE=str(Path(temporary)/'index'))
        def git(*args):
            return subprocess.run(['git', *args], cwd=upstream, env=env,
                                  capture_output=True, check=True).stdout
        git('read-tree', 'HEAD')
        for patch in patches:
            git('apply', '--cached', '--whitespace=nowarn', str(patch))
        paths = git('diff', '--cached', '--name-only', 'HEAD').decode().splitlines()
        for name in paths:
            path = upstream/name
            if not path.is_file():
                return False
            expected = git('show', ':'+name)
            if path.read_bytes().replace(b'\r\n', b'\n') != expected.replace(b'\r\n', b'\n'):
                return False
        return True

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / 'upstream/phosphobot'


def prepare():
    import json
    expected = json.loads((ROOT/'upstream.lock.json').read_text())['phosphobot']['commit']
    actual = subprocess.check_output(['git','rev-parse','HEAD'],cwd=UPSTREAM,text=True).strip()
    if actual != expected:
        raise RuntimeError('Upstream HEAD differs from the reviewed pin')
    patches = sorted((ROOT/'patches').glob('*.patch'))
    for patch in ([] if matches_complete_patch_chain(UPSTREAM, patches) else patches):
        base = ['git','apply','--whitespace=nowarn']
        check = subprocess.run(base+['--check',str(patch)],cwd=UPSTREAM,capture_output=True)
        if check.returncode == 0:
            subprocess.run(base+[str(patch)],cwd=UPSTREAM,check=True)
        elif subprocess.run(base+['--reverse','--check',str(patch)],cwd=UPSTREAM,
                            capture_output=True).returncode:
            raise RuntimeError('Patch conflict; preserve local changes: '+patch.name)
    source=ROOT/'frontend'
    if source.exists():
        shutil.copytree(source,UPSTREAM/'dashboard/src/e6',dirs_exist_ok=True)
    print('Pinned phosphobot and extension prepared')


def build_dashboard():
    dashboard=UPSTREAM/'dashboard'
    npm=shutil.which('npm.cmd') or shutil.which('npm')
    if not npm:raise RuntimeError('npm is required to build the pinned dashboard')
    subprocess.run([npm,'run','build'],cwd=dashboard,check=True)
    destination=UPSTREAM/'phosphobot/resources/dist'
    shutil.copytree(dashboard/'dist',destination,dirs_exist_ok=True)
    hashes=lambda base:{str(p.relative_to(base)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in sorted(base.rglob('*')) if p.is_file()}
    built=hashes(dashboard/'dist')
    if any(hashlib.sha256((destination/name).read_bytes()).hexdigest()!=sha for name,sha in built.items()):
        raise RuntimeError('Served dashboard differs from build output')
    (destination/'e6-build.json').write_text(json.dumps(dict(
        extensions=hashes(ROOT/'frontend'),artifacts=built),indent=2),encoding='utf-8')
    print('Dashboard built and published to the upstream resource directory')


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--build-dashboard',action='store_true')
    args=parser.parse_args()
    prepare()
    if args.build_dashboard:build_dashboard()
