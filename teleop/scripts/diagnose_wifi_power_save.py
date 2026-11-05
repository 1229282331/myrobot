"""X5-only temporary AIC8800 power-save diagnostic, with timed rollback.

This is deliberately not a launch setting. Reloading Wi-Fi disconnects SSH.
Run from an independent systemd unit after releasing all device sessions.
No robot or CAN commands are issued and no persistent network config changes.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time


def command(*args, timeout=20, check=True):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=check)


def parameters():
    root=Path('/sys/module/aic8800_fdrv/parameters')
    return {n:(root/n).read_text().strip() for n in ('ps_on','dpsm')}


def reload_wifi(values, connection):
    command('rmmod','aic8800_fdrv')
    command('modprobe','aic8800_fdrv',*(f'{n}={0 if v=="N" else 1}' for n,v in values.items()))
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        result=command('nmcli','--wait','12','connection','up','uuid',connection,timeout=15,check=False)
        if result.returncode==0:
            result=command('ping','-c','2','-W','2','192.168.1.1',timeout=6,check=False)
            if result.returncode==0:
                if parameters()!=values:raise RuntimeError('Loaded module parameters differ')
                return
        time.sleep(2)
    raise RuntimeError('Wi-Fi reconnection failed')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=('apply','restore'))
    parser.add_argument('--evidence',type=Path,required=True)
    args=parser.parse_args();directory=args.evidence
    if args.mode=='apply':
        # Native socket probes and all observation/control sessions must end.
        if command('ss','-H','-tn','dst','192.168.1.4').stdout.strip():raise RuntimeError('E6 connection still active')
        if command('fuser','/dev/video0','/dev/video1',check=False).returncode==0:raise RuntimeError('Camera still occupied')
        directory.mkdir(parents=True,exist_ok=False)
        before=parameters()
        if set(before.values())-{'Y','N'}:raise RuntimeError('Unknown driver parameter format')
        connection=command('nmcli','-g','GENERAL.CON-UUID','device','show','wlan0').stdout.strip()
        if len(connection)!=36:raise RuntimeError('No active saved wlan0 connection')
        state=dict(original=before,connection=connection,applied=False,restored=False,
                   temporary=True,robot_commands=0,can_tx=0)
        (directory/'state.json').write_text(json.dumps(state,indent=2))
        # Independent timer survives loss of our SSH control connection.
        command('systemd-run','--unit=e6-wifi-ps-rollback','--on-active=5m','python3',
                str(Path(__file__).resolve()),'restore','--evidence',str(directory))
        try:
            reload_wifi({'ps_on':'N','dpsm':'N'},connection)
            state.update(applied=True,loaded=parameters())
        except Exception as exc:
            state['error']=str(exc)
            reload_wifi(before,connection)
            state.update(restored=True,loaded=parameters())
            raise
        finally:(directory/'state.json').write_text(json.dumps(state,indent=2))
    else:
        state=json.loads((directory/'state.json').read_text())
        if not state['restored']:
            reload_wifi(state['original'],state['connection'])
            state.update(restored=True,loaded=parameters())
            (directory/'state.json').write_text(json.dumps(state,indent=2))
        command('systemctl','stop','e6-wifi-ps-rollback.timer',check=False)
    print(json.dumps(state),flush=True)


if __name__=='__main__':main()
