"""Owned existing WSL SSH byte forwarder; receives secrets only over stdin."""
import json
import os
import sys
from e6_teleop.tunnel import Tunnel


def main():
    startup=json.loads(sys.stdin.readline());password=startup.pop('password',None)
    if password:os.environ['E6_X5_PASSWORD']=password
    profile=startup.get('acceptance_profile')
    if profile:
        # Parent verified immutable source/device/evidence using Windows clocks.
        if profile['pc_transport']['platform']!='win32-wsl-ssh':raise ValueError('Split SSH acceptance required')
        startup['acceptance_profile']=dict(profile,pc_transport=dict(profile['pc_transport'],platform='linux'))
    tunnel=Tunnel(**startup)
    try:
        tunnel.start()
        print(json.dumps(dict(event='ready',pc_tcp_profile=tunnel.pc_tcp_profile,
                              control_transport_profile=tunnel.control_transport_profile,
                              telemetry_transport_profile=tunnel.telemetry_transport_profile)),flush=True)
        # EOF only releases the resources created by this child and owned X5 session.
        if sys.stdin.read():raise ValueError('Unexpected SSH runtime input after startup')
    except Exception as exc:
        error=str(exc)
        for secret in (password,startup.get('control_token')):
            if secret:error=error.replace(secret,'[secret]')
        print(json.dumps(dict(event='failure',error=error)),flush=True)
        raise
    finally:print(json.dumps(dict(event='closed',result=tunnel.close())),flush=True)


if __name__=='__main__':main()
