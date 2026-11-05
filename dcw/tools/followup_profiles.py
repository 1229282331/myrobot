"""Explicit, bounded stream-profile diagnostics; no device-property setters.

Uses already enumerated profiles to compare IR scaling and RGB delivery.
All measurements inherit healthcheck's receive-time and scene caveats.
"""
import argparse
from datetime import datetime
import json

from healthcheck import PRESETS, capture
from obsdk import ROOT


PROFILES = {
    'ir512': {'ir': (512, 384, 10, 30)},
    'ir1024': {'ir': (1024, 768, 10, 10)},
    'ir1280': {'ir': (1280, 800, 10, 10)},
    'color_vga': {'color': (640, 360, 5, 30)},
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preset', choices=PROFILES, required=True)
    parser.add_argument('--seconds', type=int, default=20)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error('seconds must be 1..120')
    PRESETS.update(PROFILES)
    run = ROOT / 'captures' / ('followup_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + args.preset)
    run.mkdir(parents=True)
    print(f'Results: {run}', flush=True)
    report = capture(run, args.preset, args.seconds)
    (run / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: report.get(k) for k in ('status', 'error', 'profiles', 'streams', 'cleanup_errors')}, indent=2), flush=True)
    return int(report['status'] != 'completed')


if __name__ == '__main__':
    raise SystemExit(main())
