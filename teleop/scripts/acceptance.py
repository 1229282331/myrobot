"""Simulation-only, real two-process WebSocket acceptance; never imports device tools."""
import argparse
import asyncio
import json
from pathlib import Path
import time
import httpx
import numpy as np
import websockets


class Controller:
    def __init__(self, ws): self.ws, self.status, self.seq = ws, None, 0
    async def hello(self):
        reply=json.loads(await self.ws.recv())
        assert reply["ok"], reply
        self.status=reply["status"]
    async def rpc(self, data, expect=True):
        await self.ws.send(json.dumps(data))
        reply=json.loads(await asyncio.wait_for(self.ws.recv(), 3))
        if "status" in reply: self.status=reply["status"]
        assert reply["ok"] == expect, reply
        return reply
    async def input(self, **updates):
        self.seq+=1
        data=dict(operation="input",seq=self.seq,lease=self.status["lease"],source="keyboard",identity="simulation acceptance",
                  connected=True,focused=True,neutral=True,deadman=False,fine=False,stop=False,frame="base",mode="translation",
                  twist=[0]*6,gripper_rate=0,raw=dict(test_only=True))
        data.update(updates)
        return await self.rpc(data)
    async def arm(self):
        await self.input()
        await self.rpc(dict(operation="arm"))
        assert self.status["armed"]


async def connect():
    ws=await websockets.connect("ws://127.0.0.1:8765/ws/control",proxy=None)
    c=Controller(ws); await c.hello(); return c


async def main(args):
    evidence=Path(args.evidence).resolve(); evidence.mkdir(parents=True,exist_ok=True)
    checks=[]; episodes=[]
    c=await connect()
    try:
        await c.arm()
        async with websockets.connect("ws://127.0.0.1:8765/ws/control",proxy=None) as second:
            rejected=json.loads(await second.recv()); assert not rejected["ok"]
        checks.append("second controller rejected")
        await c.input()
        lease=c.status["lease"]
        duplicate=dict(operation="input",seq=c.seq,lease=lease)
        await c.rpc(duplicate,expect=False); assert not c.status["armed"]
        checks.append("duplicate sequence stops")
        await c.arm()
        await c.rpc(dict(operation="input",seq=c.seq-1,lease=c.status["lease"]),expect=False)
        checks.append("out-of-order stops")
        await c.arm(); old=c.status["lease"]; await c.input()
        await c.rpc(dict(operation="input",seq=c.seq+1,lease=old),expect=False)
        checks.append("consumed lease rejected")
        await c.arm()
        await asyncio.sleep(.3)
        await c.rpc(dict(operation="input",seq=c.seq+1,lease=c.status["lease"]),expect=False)
        assert not c.status["armed"]
        checks.append("250ms stale lease stops")
        await c.arm(); await c.input(focused=False); assert not c.status["armed"]
        checks.append("focus loss stops")
        await c.arm(); await c.input(connected=False); assert not c.status["armed"]
        checks.append("device loss stops")
        await c.ws.close()
        await asyncio.sleep(.15)
        c=await connect()
        assert not c.status["armed"]
        await c.rpc(dict(operation="arm"),expect=False)
        checks.append("disconnect reconnect requires fresh neutral and explicit arm")

        for outcome in ("success","failure","aborted"):
            await c.arm()
            start=await c.rpc(dict(operation="record/start",task=f"模拟闭环验收 {outcome}：增量末端与连续夹爪（软件合成输入）"))
            name=start["recording"]["episode"]
            began=time.perf_counter()
            for tick in range(round(args.seconds*20)):
                # Bounded alternation, small Cartesian deltas; this is test input, no replay API.
                phase=tick%160
                sign=1 if phase<80 else -1
                if tick < 200:
                    twist=[sign*.002,0,0,0,0,0]; mode="translation"
                elif tick < 400:
                    twist=[0,0,0,0,0,sign*.01]; mode="rotation"
                else:
                    twist=[0,0,sign*.002,0,0,0]; mode="translation"
                # Neutral mode transition before the first nonzero command.
                if tick in (200,400):
                    await c.input(mode=mode)
                else:
                    await c.input(twist=twist,mode=mode,deadman=True,neutral=False,gripper_rate=sign*.025,fine=(tick%80>=40))
                assert c.status["armed"], c.status
                await asyncio.sleep(max(0,began+(tick+1)*.05-time.perf_counter()))
            await c.input()
            finish=await c.rpc(dict(operation="record/finish",result=outcome))
            episodes.append(dict(id=name,result=outcome,elapsed_s=time.perf_counter()-began,samples=finish["recording"]["samples"]))
            print(json.dumps(episodes[-1]),flush=True)
    finally:
        await c.ws.close()
    async with httpx.AsyncClient(base_url="http://127.0.0.1:8765",trust_env=False) as http:
        for episode in episodes:
            detail=(await http.get(f"/api/episodes/{episode['id']}")).json()
            rows=detail["rows"]
            assert detail["summary"]["complete"] and len(rows)>=int(args.seconds*19)
            differences=sum(np.max(np.abs(np.asarray(row["observation"]["joints_rad"])-row["effective_action"]["joints_rad"]))>1e-6 for row in rows)
            assert differences>100
            for row,next_row in zip(rows,rows[1:]):
                np.testing.assert_allclose(row["effective_action"]["joints_rad"],next_row["observation"]["joints_rad"],atol=1e-12)
            episode["moving_frames_with_distinct_action"]=int(differences)
        abort=episodes[-1]
        response=await http.post(f"/api/export/{abort['id']}")
        assert response.status_code==400
        checks.append("aborted raw preserved and default export rejected")
    report=dict(simulation_only=True,hardware_connections=False,checks=checks,episodes=episodes)
    (evidence/"acceptance.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(__doc__); p.add_argument("--seconds",type=float,default=30); p.add_argument("--evidence",default="docs/evidence/teleop_simulation_20261004")
    asyncio.run(main(p.parse_args()))
