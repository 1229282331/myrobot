"""One temporary WSL/Linux route to X5; no Windows/global TCP configuration.

Run create/remove as root, before/after the owned PC workbench. Existing routes
and changed ownership records are never overwritten. No device IO is issued.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from e6_teleop.pc_tcp_latency import minimum_rto_ms

PEER='192.168.1.13'


def command(*args):
    return subprocess.run(args,capture_output=True,text=True,timeout=3,check=True).stdout


def exact():return json.loads(command('ip','-j','route','show','exact',PEER+'/32'))


def inactive():
    for port in ('8765','8770','8771'):
        if command('ss','-Hlnpt','sport','=',':'+port).strip():
            raise ValueError('Owned PC workbench/tunnel still listening; refuse route mutation')


def owned(routes, selected):
    if len(routes)!=1:return False
    route=routes[0]
    return (route.get('dst') in (PEER,PEER+'/32')
            and route.get('dev')=='eth0' and str(route.get('protocol'))=='242'
            and route.get('gateway')==selected['gateway']
            and route.get('prefsrc')==selected['prefsrc']
            and minimum_rto_ms(route)==20)


def create(directory):
    owner=directory/'owner.json'
    inactive()
    if exact():raise ValueError('Existing X5 peer host route; refuse replacement')
    if owner.exists() or (directory/'intent.json').exists():raise ValueError('New route evidence required')
    selected=json.loads(command('ip','-j','route','get',PEER))
    if len(selected)!=1 or selected[0].get('dev')!='eth0' or not selected[0].get('gateway') or not selected[0].get('prefsrc'):
        raise ValueError('Expected WSL eth0 gateway route')
    row=selected[0];directory.mkdir(parents=True,exist_ok=True)
    boot=Path('/proc/sys/kernel/random/boot_id')
    intent=dict(peer=PEER,original=selected,temporary=True,device_commands=0,
                wsl_boot_id=boot.read_text().strip() if boot.exists() else None)
    (directory/'intent.json').write_text(json.dumps(intent,indent=2),encoding='utf-8')
    added=False
    try:
        command('ip','route','add',PEER+'/32','via',row['gateway'],'dev','eth0',
                'src',row['prefsrc'],'proto','242','rto_min','20ms')
        added=True
        created=exact()
        if not owned(created,row):raise ValueError('Created route readback does not match requested ownership/configuration')
        owner.write_text(json.dumps(dict(intent,created=created),indent=2),encoding='utf-8')
    except BaseException as exc:
        # Never leave an unrecorded mutation, or delete a route changed by others.
        rollback=dict(error=str(exc),restored=False,manual_recovery_required=True)
        try:
            current=exact();rollback['readback']=current
            if added and owned(current,row):
                command('ip','route','del',PEER+'/32','via',row['gateway'],'dev','eth0',
                        'src',row['prefsrc'],'proto','242')
                rollback['restored']=not exact()
                rollback['manual_recovery_required']=not rollback['restored']
        except Exception as failure:rollback['rollback_error']=str(failure)
        (directory/'create_failure.json').write_text(json.dumps(rollback,indent=2),encoding='utf-8')
        raise


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('mode',choices=('create','audit','remove'))
    p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args()
    if sys.platform!='linux':raise ValueError('Linux PC only')
    directory=a.evidence.resolve();owner=directory/'owner.json'
    if a.mode=='create':
        if os.geteuid()!=0:raise ValueError('Explicit WSL root invocation required')
        create(directory)
    elif a.mode=='remove':
        if os.geteuid()!=0:raise ValueError('Explicit WSL root invocation required')
        inactive();saved=json.loads(owner.read_text(encoding='utf-8'))
        if saved['peer']!=PEER or exact()!=saved['created']:
            raise ValueError('Peer route changed outside this task; refuse removal')
        command('ip','route','del',PEER+'/32','dev','eth0','proto','242')
        if exact():raise ValueError('Route removal not confirmed')
        (directory/'removed.json').write_text(json.dumps(dict(restored=True,route=exact()),indent=2),encoding='utf-8')
    print(json.dumps(dict(mode=a.mode,peer=PEER,route=exact(),device_commands=0)),flush=True)


if __name__=='__main__':main()
