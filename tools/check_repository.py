"""Check the Git index (or a revision) for accidental data/credential uploads.

Uses only the Python standard library. Reads Git objects, not robot devices.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys


def git(*args):
    return subprocess.check_output(['git', *args])


def entries(revision):
    if revision:
        raw = git('ls-tree', '-rz', revision)
    else:
        raw = git('ls-files', '--stage', '-z')
    result = []
    for line in raw.split(b'\0'):
        if not line:
            continue
        meta, name = line.split(b'\t', 1)
        fields = meta.split()
        mode, oid = fields[0], fields[2] if revision else fields[1]
        if not revision and fields[2] != b'0':
            raise ValueError('Resolve index conflicts before publishing')
        result.append((mode, oid.decode(), name.decode('utf-8')))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision', help='Check a tree/commit; default: the staged index')
    parser.add_argument('--pre-push', action='store_true', help='Validate refs from the Git pre-push hook')
    args = parser.parse_args()
    if args.pre_push:
        policy = json.loads(Path(__file__).with_name('repository-policy.json').read_text())
        for row in sys.stdin:
            local_ref, local_oid, remote_ref, remote_oid = row.split()
            if set(local_oid) == {'0'}:
                continue
            roots = set(git('rev-list', '--max-parents=0', local_oid).decode().splitlines())
            if roots.intersection(policy['retired_history_roots']):
                print(f'Refusing archived experiment history: {local_ref}', file=sys.stderr)
                return 1
            if subprocess.call([sys.executable, __file__, '--revision', local_oid]):
                return 1
        return 0
    rows = entries(args.revision)
    policy_name = 'tools/repository-policy.json'
    policy_oid = next(oid for mode, oid, name in rows if name == policy_name)
    policy = json.loads(git('cat-file', 'blob', policy_oid))
    retained = set(policy['retained_files'])
    denied_top = set(policy['local_only_roots'])
    errors = []
    total = 0
    largest = 0
    hashes = {}
    files = {}
    signatures = {
        'private key': rb'-----BEGIN (?:RSA |EC |DSA |OPENSSH )?PRIVATE KEY-----\r?\n[A-Za-z0-9+/=]{20}',
        'GitHub token': rb'\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})\b',
        'AWS access key': rb'\bAKIA[A-Z0-9]{16}\b',
        'API key': rb'\bsk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{40,}\b',
    }
    proc = subprocess.Popen(['git', 'cat-file', '--batch'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    try:
        for mode, oid, name in rows:
            if mode == b'160000':
                continue
            parts = PurePosixPath(name).parts
            if name not in retained:
                if parts[0] in denied_top or any(p in {'evidence', 'research', 'node_modules', 'runtime', '__pycache__', '.git'} or p.startswith(('.venv', 'workspace-before-')) for p in parts):
                    errors.append(f'Local data path: {name}')
            if PurePosixPath(name).suffix.lower() in {'.key', '.p12', '.pfx', '.pem'} or PurePosixPath(name).name in {'id_rsa', 'id_ed25519', '.env'}:
                errors.append(f'Credential filename: {name}')
            proc.stdin.write(oid.encode() + b'\n')
            proc.stdin.flush()
            head = proc.stdout.readline().split()
            if len(head) != 3 or head[1] != b'blob':
                raise ValueError(f'Missing Git blob: {name}')
            size = int(head[2])
            content = proc.stdout.read(size)
            if proc.stdout.read(1) != b'\n' or len(content) != size:
                raise ValueError('Truncated Git object stream')
            total += size
            largest = max(largest, size)
            hashes[name] = hashlib.sha256(content).hexdigest()
            if size > policy['max_file_bytes']:
                errors.append(f'Oversized file ({size} bytes): {name}')
            for label, pattern in signatures.items():
                if re.search(pattern, content):
                    errors.append(f'Possible {label}: {name}')
            if name.endswith('assets/workspace/manifest.json') or name == policy['release_manifest'] or name == 'teleop-platform/config/e6-controller-model/profile.json':
                files[name] = json.loads(content)
    finally:
        proc.stdin.close()
        proc.stdout.close()
        proc.wait()
    if total > policy['max_tree_bytes']:
        errors.append(f'Tree exceeds repository budget: {total} bytes')
    for name in retained:
        if name not in hashes:
            errors.append(f'Missing retained dependency: {name}')
    for name, manifest in files.items():
        base = str(PurePosixPath(name).parent)
        for child, digest in manifest.get('files', {}).items():
            if hashes.get(base + '/' + child) != digest:
                errors.append(f'Missing or changed asset: {base}/{child}')
        if name.endswith('e6-controller-model/profile.json'):
            for child, field in [('e6.urdf', 'urdf_sha256'), ('e6.srdf', 'srdf_sha256')]:
                if field in manifest and hashes.get(base + '/' + child) != manifest[field]:
                    errors.append(f'Controller model hash differs: {base}/{child}')
        if 'model_id' in manifest:
            payload = {k: v for k, v in manifest.items() if k != 'model_id'}
            if hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest() != manifest['model_id']:
                errors.append('Workspace model identity differs')
    if policy['release_manifest'] not in files:
        errors.append('Missing current native release manifest')
    print(json.dumps({'files': len(hashes), 'bytes': total, 'largest_bytes': largest,
                      'submodules': sum(mode == b'160000' for mode, oid, name in rows),
                      'errors': errors}, ensure_ascii=True, indent=2))
    return bool(errors)


if __name__ == '__main__':
    sys.exit(main())
