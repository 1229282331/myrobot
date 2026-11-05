"""Publish/roll back v292 static assets after Stop and leaving Control.

Does not restart the platform, create a control connection, or send device commands.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import time
import urllib.request

root = Path(__file__).resolve().parents[2]
platform = root / 'teleop-platform'
archive = root / 'output/prepare-v292'
evidence = root / 'evidence/prepare-v292'


def hashes(base):
    return {p.relative_to(base).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(base.rglob('*')) if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rollback', action='store_true')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    package = archive / ('baseline' if args.rollback else 'release')
    manifest = json.loads((package / 'sha256.json').read_text(encoding='utf8'))
    for name, sha in manifest.items():
        assert hashlib.sha256((package / name).read_bytes()).hexdigest() == sha, name
    source = package / 'dist'
    expected = hashes(source)
    assert 'index.html' in expected and 'e6-build.json' in expected
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    states = []
    for _ in range(20):
        with opener.open('http://127.0.0.1:8765/e6/state', timeout=3) as response:
            state = json.load(response)
        assert not state['ready'] and state['stationary'], 'Control must be stopped'
        assert state['recording']['phase'] in ('idle', 'complete', 'aborted', 'verified'), 'Recording not complete'
        assert state['feedback']['connected'] and state['feedback']['age_ms'] <= 100, 'Stale feedback'
        status = state['feedback']['status']
        assert not status['error_status'] and not status['collision_state'], 'Device fault'
        assert not state.get('stopping') and not state.get('stop_uncertain'), 'Stop incomplete'
        states.append(state)
        time.sleep(.1)
    assert len({s['feedback']['sequence'] for s in states}) >= 5, 'Repeated cached states'
    command_ids = {s['feedback']['status']['current_command_id'] for s in states}
    assert len(command_ids) == 1, 'Command changed during preflight'
    action = 'rollback' if args.rollback else 'publish'
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence / f'{action}-preflight.json').write_text(json.dumps(states, indent=2), encoding='utf8')
    if not args.check:
        destination = platform / 'upstream/phosphobot/phosphobot/resources/dist'
        # Keep old content-addressed assets for already-open pages and rollback.
        for name in expected:
            if name in ('index.html', 'e6-build.json'):
                continue
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / name, target)
        for name in ('e6-build.json', 'index.html'):
            temporary = destination / (name + '.v292.tmp')
            shutil.copyfile(source / name, temporary)
            os.replace(temporary, destination / name)
        assert all(hashlib.sha256((destination / n).read_bytes()).hexdigest() == sha
                   for n, sha in expected.items())
        (evidence / f'{action}-result.json').write_text(json.dumps({
            'artifacts': expected, 'command_id': state['feedback']['status']['current_command_id'],
            'static_only': True, 'time_ns': time.time_ns()}, indent=2), encoding='utf8')
    print(json.dumps({'checked': True, 'published': not args.check, 'rollback': args.rollback,
                      'files': len(expected), 'command_id': state['feedback']['status']['current_command_id']}))


if __name__ == '__main__':
    main()
