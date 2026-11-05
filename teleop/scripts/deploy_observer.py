"""Deploy only our X5 observer and its pinned WebSocket dependency. No acquisition."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import paramiko


def main():
    p=argparse.ArgumentParser();p.add_argument('--host',required=True);p.add_argument('--user',default='root');p.add_argument('--remote-root',default='/root/e6-observer-20261004');p.add_argument('--skip-dependency-install',action='store_true');p.add_argument('--evidence',type=Path);a=p.parse_args()
    client=paramiko.SSHClient();client.load_system_host_keys();client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(a.host,username=a.user,password=os.environ.get('E6_X5_PASSWORD'),timeout=8)
        root=shlex.quote(a.remote_root)
        _,stdout,stderr=client.exec_command(f'mkdir -p {root}/device_gateway/e6_observer {root}/deps {root}/config')
        if stdout.channel.recv_exit_status():raise RuntimeError(stderr.read().decode())
        sources={}
        with client.open_sftp() as sftp:
            config=Path(__file__).resolve().parents[1]/'config/gripper_profiles.json'
            sftp.put(str(config),a.remote_root+'/config/gripper_profiles.json')
            sources['config/gripper_profiles.json']=hashlib.sha256(config.read_bytes()).hexdigest()
            for file in (Path(__file__).resolve().parents[1]/'device_gateway'/'e6_observer').glob('*.py'):
                relative='device_gateway/e6_observer/'+file.name
                sftp.put(str(file),a.remote_root+'/'+relative);sources[relative]=hashlib.sha256(file.read_bytes()).hexdigest()
        _,stdout,stderr=client.exec_command(f'mkdir -p {root}/device_gateway/omnipicker/tools')
        if stdout.channel.recv_exit_status():raise RuntimeError(stderr.read().decode())
        with client.open_sftp() as sftp:
            local=Path(__file__).resolve().parents[2]/'omnipicker'/'tools'/'verify_can_control.py'
            sftp.put(str(local),a.remote_root+'/device_gateway/omnipicker/tools/verify_can_control.py')
            sources['device_gateway/omnipicker/tools/verify_can_control.py']=hashlib.sha256(local.read_bytes()).hexdigest()
            for relative,expected in sources.items():
                with sftp.open(a.remote_root+'/'+relative,'rb') as deployed:
                    if hashlib.sha256(deployed.read()).hexdigest()!=expected:raise ValueError('Deployment checksum mismatch: '+relative)
        if a.skip_dependency_install:
            check="import websockets,zstandard; assert websockets.__version__=='15.0.1'; assert zstandard.__version__=='0.25.0'; print('Pinned dependencies verified')"
            command=f'env PYTHONPATH={root}/deps python3 -c '+shlex.quote(check)
        else:command=f'python3 -m pip install --disable-pip-version-check --no-deps --target {root}/deps websockets==15.0.1 zstandard==0.25.0'
        _,stdout,stderr=client.exec_command(command)
        print(stdout.read().decode())
        if stdout.channel.recv_exit_status():raise RuntimeError(stderr.read().decode())
        if a.evidence:
            a.evidence.parent.mkdir(parents=True,exist_ok=True)
            a.evidence.write_text(json.dumps(dict(remote_root=a.remote_root,host=a.host,source_sha256=sources,all_remote_hashes_verified=True,device_commands_sent=0),indent=2),encoding='utf-8')
        print('Observer deployed; no device streams or CAN commands started.')
    finally:client.close()


if __name__=='__main__':main()
