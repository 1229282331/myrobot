"""Local-only diagnostic bundle with an explicit allowlist, never SSH secrets."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('deployment.json', 'source-sha256.json', 'startup-state.json', 'result.json',
         'lifecycle.jsonl', 'ssh.log', 'local-8554.log', 'local-8765.log', 'fault-injection.json')


def export(session, output):
    session = Path(session).resolve()
    base = (ROOT/'runtime/sessions').resolve()
    if session.parent != base or not session.is_dir():
        raise ValueError('Select one local session directory')
    index = {}
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as bundle:
        for name in NAMES:
            path = session/name
            if not path.is_file():
                continue
            # No recursive traversal, environment dump, private helper, keys,
            # media or raw dataset collection. Bundle remains local.
            data = path.read_bytes()
            bundle.writestr(session.name+'/'+name, data)
            index[name] = hashlib.sha256(data).hexdigest()
        bundle.writestr(session.name+'/bundle-index.json', json.dumps(index, indent=2))
    return index


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', default='latest')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    base = ROOT/'runtime/sessions'
    session = max(base.iterdir(), key=lambda p: p.name) if args.session == 'latest' else base/args.session
    output = args.output or ROOT/'runtime'/('support-'+session.name+'.zip')
    export(session, output)
    print(output)
