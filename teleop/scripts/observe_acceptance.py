"""Real observation stability test through the PC service, never a motion test."""
import argparse
import asyncio
import base64
import hashlib
import json
from pathlib import Path
import threading
import time
import urllib.request
import uvicorn
import numpy as np
import websockets
from e6_teleop.tunnel import Tunnel
from e6_teleop.observation_service import make_app


def request(path,data=None):
    body=json.dumps(data).encode() if data is not None else None
    req=urllib.request.Request('http://127.0.0.1:8768'+path,data=body,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=3) as response:return json.load(response)


async def run(seconds,evidence):
    async with websockets.connect('ws://127.0.0.1:8768/ws/state',max_size=8*1024*1024,proxy=None) as ws:
        deadline=time.monotonic()+20
        while True:
            f=json.loads(await asyncio.wait_for(ws.recv(),3))
            if f.get('observation',{}).get('valid'):break
            if time.monotonic()>deadline:raise RuntimeError('Real observations not ready: '+str(f.get('sources')))
        first=f;start=time.monotonic();count=0;invalid=0;max_age={'robot':0,'rgb':0};max_pc_age={'robot':0,'rgb':0};record=None
        if seconds>=60:record=request('/api/record/start',{'task':'60秒真实设备观测链路验收'})
        record_finished=False
        ages={n:[] for n in ('robot','rgb','rgb_preview','depth_preview','transport')}
        previews_stale=0;media_sizes=[];previous_preview_frame=None
        while time.monotonic()-start<seconds:
            f=json.loads(await asyncio.wait_for(ws.recv(),3));count+=1
            if not f.get('observation',{}).get('valid'):invalid+=1
            for name in max_age:
                age=f.get('sources',{}).get(name,{}).get('age_ms')
                if age is not None:max_age[name]=max(age,max_age[name])
                age=f.get('sources',{}).get(name,{}).get('pc_age_upper_ms')
                if age is not None:max_pc_age[name]=max(max_pc_age[name],age)
            for name in ('robot','rgb'):
                age=f.get('sources',{}).get(name,{}).get('pc_age_upper_ms')
                if age is not None:ages[name].append(age)
            age=f.get('transport',{}).get('age_upper_ms')
            if age is not None:ages['transport'].append(age)
            for name in ('rgb','depth'):
                preview=f.get('preview',{}).get(name,{})
                age=preview.get('age_upper_ms')
                if age is not None:ages[name+'_preview'].append(age)
                if name=='rgb' and not preview.get('valid'):previews_stale+=1
            metric=f.get('stats',{}).get('media_publisher',{})
            if metric.get('frame')!=previous_preview_frame:
                media_sizes.append(metric.get('wire_bytes',0));previous_preview_frame=metric.get('frame')
            if f.get('pc_error'):
                evidence.mkdir(parents=True,exist_ok=True)
                (evidence/'failure_frame.json').write_text(json.dumps({k:v for k,v in f.items() if k!='images'},ensure_ascii=False,indent=2),encoding='utf-8')
                raise RuntimeError('PC observation error: '+f['pc_error']+'; transport='+str(f.get('transport')))
            if record and time.monotonic()-start>=60 and not record_finished:
                record=request('/api/record/finish',{'result':'observed'});record_finished=True
            if count%600==0:print(json.dumps(dict(elapsed_s=round(time.monotonic()-start,1),preview_samples=count,invalid=invalid,
                                                   actual_fps={n:f['stats'][n]['actual_fps'] for n in ('robot','rgb','depth')},media=f['stats'].get('media_publisher')),ensure_ascii=False),flush=True)
        record=request('/api/state')['recording'] if record else None
        if record and record.get('pending'):raise RuntimeError('Native media transfer still pending after stability window')
        if record and record.get('phase')!='complete':raise RuntimeError('Native media did not complete: '+str(record))
        result=dict(duration_s=time.monotonic()-start,received_preview_samples=count,invalid_required=invalid,
                    max_age_ms=max_age,max_pc_age_upper_ms=max_pc_age,first_sources={n:{k:v for k,v in s.items() if k not in ('jpeg','uint16_raw')} for n,s in first['sources'].items()},final_sources=f['sources'],
                    age_statistics_ms={n:{'median':float(np.median(v)),'p95':float(np.percentile(v,95)),'p99':float(np.percentile(v,99)),'max':max(v)} for n,v in ages.items() if v},
                    stale_rgb_previews=previews_stale,mean_preview_message_bytes=float(np.mean(media_sizes)),final_stats=f.get('stats'),recording=record,
                    pc_pipeline=f.get('pc_pipeline'),simulated=False,can_tx_count=0,arm_motion_commands=0)
        evidence.mkdir(parents=True,exist_ok=True)
        (evidence/'live_stability.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(duration_s=result['duration_s'],samples=count,invalid=invalid,recording=record),ensure_ascii=False),flush=True)
        if invalid:raise RuntimeError('Required observation became invalid')


def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='192.168.1.13');p.add_argument('--seconds',type=float,default=600);p.add_argument('--evidence',type=Path,default=Path('docs/evidence/teleop_observation_20261004'));p.add_argument('--wifi-low-latency',action='store_true');p.add_argument('--no-model-preview',action='store_true');p.add_argument('--media-hz',type=int,choices=(5,10,20),default=20);a=p.parse_args()
    tunnel=Tunnel(a.host,remote_root='/root/e6-observer-20261004',wifi_low_latency=a.wifi_low_latency,media_hz=a.media_hz)
    server=None
    try:
        tunnel.start()
        server=uvicorn.Server(uvicorn.Config(make_app(render=not a.no_model_preview,archive_gateway=f'ws://127.0.0.1:{tunnel.archive_port}'),host='127.0.0.1',port=8768,log_level='warning'))
        thread=threading.Thread(target=server.run,daemon=True);thread.start()
        for _ in range(100):
            if server.started:break
            time.sleep(.05)
        asyncio.run(run(a.seconds,a.evidence))
    finally:
        if server:server.should_exit=True;thread.join(10)
        tunnel.close()
        a.evidence.mkdir(parents=True,exist_ok=True)
        (a.evidence/'gateway_process_log.txt').write_text(''.join(getattr(tunnel,'logs',[])),encoding='utf-8')


if __name__=='__main__':main()
