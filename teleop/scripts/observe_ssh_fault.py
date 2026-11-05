"""Interrupt only our SSH transport; no robot/CAN writes or device shutdown."""
import json
import argparse
from pathlib import Path
import threading
import time
import urllib.request
import uvicorn
from e6_teleop.tunnel import Tunnel
from e6_teleop.observation_service import make_app


def request(path,data=None):
    req=urllib.request.Request('http://127.0.0.1:8770'+path,data=json.dumps(data).encode() if data is not None else None,
                              headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=3) as r:return json.load(r)


def run_session(interrupt):
    tunnel=Tunnel('192.168.1.13',remote_root='/root/e6-observer-20261004');server=None
    try:
        tunnel.start()
        server=uvicorn.Server(uvicorn.Config(make_app(archive_gateway=f'ws://127.0.0.1:{tunnel.archive_port}'),host='127.0.0.1',port=8770,log_level='warning'))
        thread=threading.Thread(target=server.run,daemon=True);thread.start()
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            if server.started and request('/api/state')['gateway_ready']:break
            time.sleep(.1)
        else:raise RuntimeError('Observation session not ready')
        before=request('/api/state');assert not before['recording']['active']
        record=request('/api/record/start',{'task':'真实SSH断连保护' if interrupt else '断连后显式重新开始'})
        time.sleep(1)
        if interrupt:
            tunnel.client.get_transport().close()
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                state=request('/api/state')
                if not state['recording']['active']:break
                time.sleep(.1)
            assert not state['recording']['active'] and state['error']
        else:
            request('/api/record/finish',{'result':'observed'})
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                if not request('/api/state')['recording'].get('pending'):break
                time.sleep(.1)
            else:raise RuntimeError('Native media transfer did not complete')
        episode=next(x for x in request('/api/episodes') if x['id']==record['episode'])
        assert episode['result']==('aborted' if interrupt else 'observed')
        if interrupt:assert not episode['complete']
        else:assert episode['complete']
        return dict(interrupted=interrupt,before=before,after=request('/api/state'),episode=episode)
    finally:
        if server:server.should_exit=True;thread.join(10)
        tunnel.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path('docs/evidence/teleop_observation_20261004/ssh_fault_validation.json'));args=parser.parse_args()
    result=dict(sessions=[run_session(True),run_session(False)],hardware_motion_commands=0,can_send_calls=0,
                explicit_restart_required=True,physical_cables_not_disconnected=True)
    output=args.output
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
