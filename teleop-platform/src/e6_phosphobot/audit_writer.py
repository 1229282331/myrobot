"""Private audit serializer subprocess; no device imports or network access.

stdin carries Python objects from the owning Audit over its private inherited
pipe only. Never use this protocol for a file, HTTP input, or external peer.
"""
import base64
import json
import pickle
from pathlib import Path
import sys


def write_stream(directory, source, counts=None):
    if counts is None:
        counts = dict(raw=0, events=0)
    with (directory / 'raw.jsonl').open('w') as raw, (directory / 'events.jsonl').open('w', encoding='utf-8') as events:
        while True:
            kind, value = pickle.load(source)
            if kind is None:
                return counts
            if kind == 'raw':
                generation, sequence, received_ns, prefix, data = value
                raw.write(json.dumps(dict(generation=generation, sequence=sequence,
                    received_ns=received_ns, prefix_b64=base64.b64encode(prefix).decode(),
                    raw_b64=base64.b64encode(data).decode())) + '\n')
            elif kind == 'events':
                # Rendering belongs to this independent interpreter. Captured
                # clocks/feedback and event order are preserved by the pipe.
                from .state_observation import render_audit_event
                value = render_audit_event(value)
                events.write(json.dumps(value, ensure_ascii=False) + '\n')
            else:
                raise ValueError('Invalid private audit message')
            counts[kind] += 1


def main():
    result = dict(error=None, counts=dict(raw=0, events=0))
    try:
        # Keep progress when an unexpected EOF or render failure ends a
        # partial journal. Error receipts must not falsely report zero work.
        write_stream(Path(sys.argv[1]), sys.stdin.buffer, result['counts'])
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    print(json.dumps(result), flush=True)
    return int(result['error'] is not None)


if __name__ == '__main__':
    raise SystemExit(main())
