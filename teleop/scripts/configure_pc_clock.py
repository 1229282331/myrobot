"""Explicit temporary WSL VM clocksource selection while project is stopped.

This affects the existing WSL VM, not Windows or X5. Restore before ending the
test session; it is not a persistent kernel/boot configuration.
"""
import argparse
import json
import os
from pathlib import Path
import sys
from configure_pc_peer_route import inactive

ROOT=Path('/sys/devices/system/clocksource/clocksource0')
TARGET='hyperv_clocksource_tsc_page'


def main():
    p=argparse.ArgumentParser(__doc__);p.add_argument('mode',choices=('create','audit','remove'))
    p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    if sys.platform!='linux' or 'microsoft' not in Path('/proc/sys/kernel/osrelease').read_text().lower():
        raise ValueError('Existing WSL VM required')
    source=ROOT/'current_clocksource';available=(ROOT/'available_clocksource').read_text().split()
    directory=a.evidence.resolve();owner=directory/'owner.json'
    if a.mode=='create':
        if os.geteuid()!=0:raise ValueError('Explicit WSL root invocation required')
        inactive()
        if owner.exists():raise ValueError('New clock evidence required')
        original=source.read_text().strip()
        if original!='tsc' or TARGET not in available:raise ValueError('Expected original tsc and available Hyper-V reference clock')
        directory.mkdir(parents=True,exist_ok=True)
        report=dict(original=original,requested=TARGET,available=available,temporary=True,scope='existing WSL VM',device_commands=0)
        (directory/'intent.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        try:
            source.write_text(TARGET+'\n')
            report['effective']=source.read_text().strip()
            if report['effective']!=TARGET:raise ValueError('WSL clocksource readback failed')
            owner.write_text(json.dumps(report,indent=2),encoding='utf-8')
        except BaseException:
            if source.read_text().strip()==TARGET:source.write_text(original+'\n')
            (directory/'create_failure.json').write_text(json.dumps(dict(restored=source.read_text().strip()==original),indent=2),encoding='utf-8')
            raise
    elif a.mode=='remove':
        if os.geteuid()!=0:raise ValueError('Explicit WSL root invocation required')
        inactive();saved=json.loads(owner.read_text(encoding='utf-8'))
        if saved['scope']!='existing WSL VM' or saved['original']!='tsc' or source.read_text().strip()!=TARGET:
            raise ValueError('Clocksource changed outside task; refuse overwrite')
        source.write_text(saved['original']+'\n')
        if source.read_text().strip()!=saved['original']:raise ValueError('Clock restoration not confirmed')
        (directory/'removed.json').write_text(json.dumps(dict(restored=True,effective=source.read_text().strip()),indent=2),encoding='utf-8')
    print(json.dumps(dict(mode=a.mode,effective=source.read_text().strip(),device_commands=0)))


if __name__=='__main__':main()
