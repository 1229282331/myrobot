"""Launch only a source/configuration-matched, accepted operator workbench."""
import argparse
import getpass
import os
from pathlib import Path
import socket
import sys
import threading
import time
import urllib.request
import webbrowser

from .acceptance_profile import verify_profile
from .model import ROOT


def arguments(profile_path):
    path=Path(profile_path).resolve()
    profile=verify_profile(path)
    values=['serve','--backend','hardware-control','--control-stage','live','--controller-role','operator',
        '--ssh-host','192.168.1.13','--remote-root','/root/e6-control-20261005',
        '--acceptance-profile',str(path),'--media-hz',str(profile['media_hz']),
        '--feedback-framing',profile.get('feedback_framing','strict'),
        '--recovery-policy',profile.get('recovery_policy','manual')]
    values+=['--gripper-profile',profile.get('gripper_command_profile','plan32')]
    if profile.get('kinematics_kind')=='controller-local':
        bundle=(path.parent/profile['kinematics_profile_file']).resolve()
        from hashlib import sha256
        if sha256(bundle.read_bytes()).hexdigest()!=profile['kinematics_profile_sha256']:
            raise ValueError('Accepted controller-local bundle changed')
        values+=['--kinematics','controller-local','--kinematics-profile',str(bundle)]
    if profile.get('wifi_low_latency'):values+=['--wifi-low-latency']
    for field,flag in [('wifi_reorder_ms','--wifi-reorder-ms'),('ssh_rto_min_ms','--ssh-rto-min-ms'),('e6_rto_min_ms','--e6-rto-min-ms')]:
        if profile.get(field) is not None:values += [flag,str(profile[field])]
    if profile['pc_transport']['platform']=='win32-wsl-ssh':
        values+=['--ssh-runtime','wsl','--pc-rto-min-ms','20']
    return values


def authenticate():
    # Reuse existing trusted keys/agent or the process environment. A fallback
    # password is entered privately and never written to disk or command args.
    import paramiko
    client=paramiko.SSHClient();client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        try:client.connect('192.168.1.13',username='root',password=os.environ.get('E6_X5_PASSWORD'),timeout=8)
        except paramiko.AuthenticationException:
            if not sys.stdin.isatty():raise RuntimeError('X5 authentication required; use a terminal or an existing SSH key')
            password=getpass.getpass('X5 SSH password (not saved): ')
            client.connect('192.168.1.13',username='root',password=password,timeout=8)
            os.environ['E6_X5_PASSWORD']=password
    finally:client.close()


def open_when_ready(cancel):
    for _ in range(200):
        if cancel.wait(.1):return
        try:
            with urllib.request.urlopen('http://127.0.0.1:8765/api/state',timeout=.5) as response:
                if response.status==200:
                    webbrowser.open('http://127.0.0.1:8765/');return
        except (OSError,TimeoutError):continue


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--profile',type=Path,default=ROOT/'config/operator_acceptance.json')
    parser.add_argument('--check',action='store_true',help='Offline acceptance/source validation; no SSH or device requests')
    parser.add_argument('--no-browser',action='store_true')
    args=parser.parse_args()
    try:values=arguments(args.profile)
    except (ValueError,OSError,KeyError) as exc:parser.error('Operator acceptance is not ready: '+str(exc))
    if args.check:
        print('Acceptance profile, source SHA and fixed startup parameters verified; no devices connected.');return
    with socket.socket() as probe:
        try:probe.bind(('127.0.0.1',8765))
        except OSError:parser.error('Workbench already running on 127.0.0.1:8765; close that session first')
    authenticate()
    cancel=threading.Event()
    if not args.no_browser:threading.Thread(target=open_when_ready,args=(cancel,),daemon=True).start()
    original=sys.argv
    try:
        sys.argv=[original[0],*values]
        from .cli import main as serve
        serve()
    finally:cancel.set();sys.argv=original


if __name__=='__main__':main()
