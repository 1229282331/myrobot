"""Protocol faults over the real SSH bridge, only with the write-proof fixture.

This never certifies physical stopping or opens hardware-control/live.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import websockets
from e6_teleop.control_transport import StreamingGatewayConnection


async def run(args):
    fixtures=json.loads(args.fixtures.read_text());neutral=fixtures['neutral']['keyboard']
    token=os.environ['E6_TELEOP_HARNESS_TOKEN'];results=[];seq=0
    headers={'X-E6-Control-Token':token,'X-E6-Control-Role':'harness'}
    async def ignored(_):raise AssertionError('All diagnostic calls must be correlated')
    async def open_bridge():
        ws=await websockets.connect(args.url+'/ws/control',compression=None,proxy=None,additional_headers=headers)
        hello=json.loads(await ws.recv())
        if not hello.get('ok'):await ws.close();raise RuntimeError(hello.get('error'))
        return StreamingGatewayConnection(ws,hello,ignored)
    bridge=await open_bridge()
    async def input(**kw):
        nonlocal seq
        seq+=1
        return await bridge.rpc(dict(neutral,seq=seq,lease=bridge.status['lease'],**kw))
    async def prepare():
        response=await input()
        assert response['ok'] and not response['status']['armed'],'Fresh neutral alone must not rearm'
        response=await bridge.rpc({'operation':'arm'})
        assert response['ok'] and response['status']['armed'],response.get('error')
    passed=False;failure=None
    try:
        for name in ('duplicate','old-lease','lease-expiry','focus','disconnected','stop','enable','initialize-gripper','raw-dashboard'):
            await prepare()
            if name=='duplicate':
                response=await bridge.rpc(dict(neutral,seq=seq,lease=bridge.status['lease']))
            elif name=='old-lease':
                response=await bridge.rpc(dict(neutral,seq=seq+1,lease='invalid-old-lease'))
            elif name=='lease-expiry':
                await input(deadman=True)
                await asyncio.sleep(.30)
                response=await input()
            elif name in ('focus','disconnected','stop'):
                response=await input(**{'focus':{'focused':False},'disconnected':{'connected':False},'stop':{'stop':True}}[name])
            else:response=await bridge.rpc({'operation':name,'command':'ServoJ(0,0,0,0,0,0)'})
            assert not response['status']['armed'],name
            if name not in ('focus','disconnected','stop'):assert response['ok'] is False,name
            results.append(dict(case=name,passed=True,armed=False,reason=response['status']['reason']))
        await prepare();await bridge.close();await asyncio.sleep(.1)
        bridge=await open_bridge();seq=0
        assert not bridge.status['armed']
        await prepare()
        results.append(dict(case='disconnect-and-explicit-reprepare',passed=True,automatic_rearm=False))
        await bridge.rpc({'operation':'stop'});passed=True
    except Exception as exc:failure=str(exc);raise
    finally:
        await bridge.close()
        args.evidence.mkdir(parents=True,exist_ok=True)
        result=dict(passed=passed,failure=failure,test_fixture=True,eligible_for_hardware_acceptance=False,
                    cases=results,physical_stopping_verified=False)
        (args.evidence/'client_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--url',default='ws://127.0.0.1:8878')
    p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True)
    asyncio.run(run(p.parse_args()))
