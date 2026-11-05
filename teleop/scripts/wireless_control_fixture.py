"""Real SSH/Wi-Fi and production guardian/bridge, with a write-proof substitute.

Never hardware acceptance: no robot socket, camera, CAN or enabled stage exists.
The optional preview stream is synthetic traffic, not an observation record.
"""
import argparse
import asyncio
from collections import deque
import hashlib
import json
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))


class Substitute:
    def __init__(self):
        self.anchor=np.radians([-66,-60,105,-90,90,9])
        self.cancel=threading.Event();self.release_pending=False;self.events=deque(maxlen=256)
        self.gripper=None;self.motion_tx=self.stop_tx=0
    def fk(self,q,deadline):
        v=np.radians(q)-self.anchor
        return [*(np.array([0,-200,400])+v[:3]*400),*np.degrees(v[3:])]
    def ik(self,p,q,deadline):
        return np.degrees(self.anchor+np.r_[(np.asarray(p[:3])-[0,-200,400])/400,np.radians(p[3:])]).tolist()
    def state(self):
        return dict(source='TEST SUBSTITUTE, not E6',x5_receive_monotonic_ns=time.perf_counter_ns(),
                    decoded=dict(joints_rad=self.anchor.tolist(),q_target_deg=np.degrees(self.anchor).tolist(),
                        robot_mode=4,enable_status=0,error_status=0,user_index=0,tool_index=0,
                        controller_pose_raw_mm_deg=[0,-200,400,0,0,0],tool_configuration_raw_mm_deg=[0]*6,
                        user_configuration_raw_mm_deg=[0]*6,collision_state=0,pause_command_flag=0,
                        run_queued_command=0,current_command_id=0))
    def servo(self,*_):raise AssertionError('Substitute forbids all device writes')
    def grip(self,*_):raise AssertionError('Substitute forbids all device writes')
    def stop(self,*_):raise AssertionError('Substitute forbids all device writes')
    def query(self,*_):raise AssertionError('Substitute forbids all Dashboard commands')
    def close(self):pass


def server(args):
    from e6_observer.control_guardian import run_guardian,put_latest
    from websockets.sync.server import serve
    token=json.loads(sys.stdin.readline())['token']
    done=threading.Event();ready=threading.Event();metadata=queue.Queue(1);results=queue.Queue(256)
    heartbeat=SimpleNamespace(value=time.perf_counter_ns());device=Substitute()
    counts=dict(status_messages=0,candidate_bearing_status_messages=0,fatal=None)
    def eof():
        while sys.stdin.buffer.read(1):pass
        done.set()
    def upkeep():
        while not done.wait(.01):
            heartbeat.value=time.perf_counter_ns();put_latest(metadata,{'rgb':{'source':'TEST synthetic RGB clock','x5_receive_monotonic_ns':heartbeat.value}})
            while True:
                try:r=results.get_nowait()
                except queue.Empty:break
                counts['status_messages']+=1;counts['candidate_bearing_status_messages']+=int(bool((r.get('tick') or {}).get('candidate_action')))
                if r.get('fatal'):counts['fatal']=r['fatal'];done.set()
    def preview(ws):
        # Deterministic incompressible traffic; JSON/SSH compression is off.
        import base64,random
        payload=base64.b64encode(random.Random(7).randbytes(args.preview_bytes)).decode()
        deadline=time.perf_counter();seq=0
        try:
            while not done.is_set():
                ws.send(json.dumps(dict(test_fixture=True,seq=seq,payload=payload)));seq+=1
                deadline+=1/args.preview_hz
                if deadline<time.perf_counter():deadline=time.perf_counter()
                done.wait(max(0,deadline-time.perf_counter()))
        except Exception:pass
    threads=[threading.Thread(target=f,daemon=True) for f in (eof,upkeep)]
    for t in threads:t.start()
    guardian=threading.Thread(target=run_guardian,args=('substitute-only','shadow','harness',token,{},metadata,results,heartbeat,done,ready),
        kwargs=dict(link_factory=lambda _:device,feedback_source=device.state,listen_port=args.port),daemon=True)
    guardian.start()
    try:
        if not ready.wait(3):raise RuntimeError('Substitute guardian not ready')
        with serve(preview,'127.0.0.1',args.preview_port,compression=None) as stream:
            threading.Thread(target=stream.serve_forever,daemon=True).start()
            print('Write-proof substitute ready, no hardware IO',flush=True)
            while not done.wait(.05):pass
            stream.shutdown()
    finally:
        done.set();guardian.join(3)
        args.evidence.mkdir(parents=True,exist_ok=True)
        (args.evidence/'server_summary.json').write_text(json.dumps(dict(test_fixture=True,
            eligible_for_hardware_acceptance=False,**counts,motion_tx=device.motion_tx,stop_tx=device.stop_tx,can_tx=0,
            guardian_exited=not guardian.is_alive(),preview_bytes=args.preview_bytes,preview_hz=args.preview_hz),indent=2))


async def client(args):
    import os
    import websockets
    from e6_teleop.control_transport import StreamingGatewayConnection
    fixtures=json.loads(args.fixtures.read_text());sha=hashlib.sha256((Path(__file__).resolve().parents[1]/'web/src/input.ts').read_bytes()).hexdigest()
    if sha!=fixtures['production_input_sha256']:raise ValueError('Production mapper changed')
    samples=[];request_id=0;seq=0;outstanding={};last_send=None;gaps=[];preview_count=0;preview_bytes=0
    error=None;cases=[];skipped=0;status=None;success=False
    async def read_preview():
        nonlocal preview_count,preview_bytes
        async with websockets.connect(args.preview_url,compression=None,proxy=None,max_queue=2) as ws:
            async for message in ws:
                if not json.loads(message).get('test_fixture'):raise ValueError('Expected synthetic traffic')
                preview_count+=1;preview_bytes+=len(message)
    preview_task=asyncio.create_task(read_preview()) if args.preview_url else None
    async def notify(reply):
        nonlocal error,status
        sent,op=outstanding.pop(reply['request_id']);now=time.perf_counter_ns()
        samples.append(dict(operation=op,pc_sent_ns=sent,pc_reply_ns=now,round_trip_ms=(now-sent)/1e6,
                            ok=reply['ok'],x5=reply.get('rpc_timing')))
        if not reply['ok']:error=RuntimeError(reply.get('error','Rejected'))
        status=reply.get('status',status)
    token=os.environ['E6_TELEOP_HARNESS_TOKEN'];begin=time.monotonic();bridge=None
    async def phase(seconds,row,armed=False):
        nonlocal seq,request_id,last_send,skipped
        deadline=time.monotonic();end=deadline+seconds
        while time.monotonic()<end:
            if error:raise error
            if armed and not bridge.status['armed']:raise RuntimeError(bridge.status['reason'])
            if preview_task and preview_task.done():await preview_task;raise RuntimeError('Synthetic preview ended')
            seq+=1;request_id+=1;now=time.perf_counter_ns()
            if last_send is not None:gaps.append((now-last_send)/1e6)
            last_send=now;outstanding[request_id]=(now,'input')
            await bridge.send(dict(row,seq=seq,lease=bridge.status['lease'],request_id=request_id))
            deadline+=.05
            if deadline<time.monotonic():skipped+=1;deadline=time.monotonic()+.05
            await asyncio.sleep(max(0,deadline-time.monotonic()))
    try:
        async with websockets.connect(args.url+'/ws/control',compression=None,proxy=None,
            additional_headers={'X-E6-Control-Token':token,'X-E6-Control-Role':'harness'}) as ws:
            hello=json.loads(await ws.recv());bridge=StreamingGatewayConnection(ws,hello,notify)
            n=fixtures['neutral']['gamepad'];await phase(.2,n);r=await bridge.rpc({'operation':'arm'})
            if not r['ok']:raise RuntimeError(r['error'])
            i=0
            while time.monotonic()-begin<args.seconds:
                row=fixtures['rows'][i%len(fixtures['rows'])]
                n=dict(fixtures['neutral'][row['source']],mode=row['mode'],frame=row['frame'])
                await phase(.15,n)
                if not bridge.status['armed']:
                    r=await bridge.rpc({'operation':'arm'})
                    if not r['ok']:raise RuntimeError(r['error'])
                await phase(.35,row['input'],True);await phase(.15,n,True)
                cases.append({k:row[k] for k in ('source','mode','frame','fine','axis','sign')});i+=1
                if i%32==0:print(json.dumps(dict(test_fixture=True,cases=i,elapsed_s=round(time.monotonic()-begin,1))),flush=True)
            await bridge.rpc({'operation':'stop'});success=True
    except Exception as exc:error=exc
    finally:
        if bridge:await bridge.close()
        if preview_task:preview_task.cancel();await asyncio.gather(preview_task,return_exceptions=True)
        args.evidence.mkdir(parents=True,exist_ok=True)
        percentile=lambda values:{k:float(np.percentile(values,p)) for k,p in [('p50',50),('p95',95),('p99',99),('max',100)]} if values else {}
        result=dict(test_fixture=True,eligible_for_hardware_acceptance=False,passed=success,failure=str(error) if error else None,
            duration_s=time.monotonic()-begin,cases=cases,production_input_sha256=sha,input_packets=len(samples),
            round_trip_ms=percentile([s['round_trip_ms'] for s in samples]),input_period_ms=percentile(gaps),skipped=skipped,
            synthetic_preview_messages=preview_count,synthetic_preview_wire_bytes=preview_bytes)
        (args.evidence/'client_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        with (args.evidence/'rpc_timings.jsonl').open('w') as f:
            for sample in samples:f.write(json.dumps(sample)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='cases'},ensure_ascii=False),flush=True)
    if not success:raise RuntimeError('Wireless substitute test failed: '+str(error))


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('mode',choices=('server','client'));p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--port',type=int,default=8888);p.add_argument('--preview-port',type=int,default=8889)
    p.add_argument('--preview-bytes',type=int,default=15000);p.add_argument('--preview-hz',type=int,default=10)
    p.add_argument('--url',default='ws://127.0.0.1:8878');p.add_argument('--preview-url')
    p.add_argument('--fixtures',type=Path);p.add_argument('--seconds',type=float,default=600);a=p.parse_args()
    server(a) if a.mode=='server' else asyncio.run(client(a))
