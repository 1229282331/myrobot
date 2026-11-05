"""Full phosphobot app with the E6 intent extension.

This first entry point exposes only upstream simulation. Real-device activation
will use an independent X5 gateway, never the simulation methods below.
"""
import argparse
import asyncio
from dataclasses import asdict
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from .intent import IntentSession, Limits


def install_simulation(app):
    from phosphobot.robot import get_rcm
    router=APIRouter()
    owner=None
    limits=Limits(linear_m_s=.01,angular_rad_s=np.deg2rad(5),joint_rad_s=np.deg2rad(5),gripper_rate_s=.1)

    @router.get('/e6/capabilities')
    def capabilities():
        return dict(enabled=True,backend='上游SO100模拟（不连接E6）',synthetic_keys=True,**asdict(limits))

    @router.websocket('/e6/control')
    async def control(ws:WebSocket):
        nonlocal owner
        await ws.accept()
        if owner is not None:
            await ws.send_json(dict(kind='state',state='暂挂',ready=False,echo_ns=0,reason='已有其他控制者'))
            await ws.close(code=1008)
            return
        owner=ws
        session=IntentSession(limits)
        try:
            robot=await get_rcm().get_robot(0)
        except Exception:
            owner=None
            await ws.close(code=1011)
            raise
        grip=1.
        pending_stop=False

        async def receiver():
            async for data in ws.iter_json():
                try:
                    session.receive(data)
                except Exception as exc:
                    session.suspend('输入格式错误: '+str(exc))

        async def pump():
            nonlocal grip,pending_stop
            while True:
                started=time.perf_counter()
                intent=session.active()
                if session.stop_requested:
                    # SO100 simulation has immediate targets, no queued motion.
                    session.stop_requested=False
                    pending_stop=False
                position,euler=robot.forward_kinematics()
                if intent is not None:
                    if any(intent.joint_rad_s):
                        q=robot.read_joints_position(source='sim')
                        robot.set_motors_positions(q+np.array(intent.joint_rad_s)*.05)
                    elif any(intent.linear_m_s+intent.angular_rad_s):
                        rotation=Rotation.from_euler('xyz',euler)
                        delta=np.array(intent.linear_m_s)*.05
                        dr=Rotation.from_rotvec(np.array(intent.angular_rad_s)*.05)
                        if intent.frame=='tool':
                            delta=rotation.apply(delta)
                            target_rotation=rotation*dr
                        else:
                            target_rotation=dr*rotation
                        await robot.move_robot_absolute(position+delta,target_rotation.as_euler('xyz'))
                    if intent.gripper_rate_s:
                        grip=float(np.clip(grip+intent.gripper_rate_s*.05,0,1))
                        robot.control_gripper(grip)
                position,_=robot.forward_kinematics()
                await ws.send_json(dict(kind='state',echo_ns=session.clock_tick(),
                    state='可控制' if session.ready else '暂挂' if session.reason else '可准备',
                    ready=session.ready,reason=session.reason,dropped=session.dropped,
                    actual=dict(joints_rad=robot.read_joints_position(source='sim').tolist(),
                                position_m=position.tolist(),gripper=grip)))
                await asyncio.sleep(max(0,.05-(time.perf_counter()-started)))

        tasks=[asyncio.create_task(receiver()),asyncio.create_task(pump())]
        try:
            done,_=await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect,RuntimeError):
            pass
        finally:
            session.suspend('connection_closed')
            for task in tasks: task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)
            owner=None

    # Install before upstream's SPA fallback, without replacing its API or UI.
    app.router.routes[0:0]=router.routes


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--backend',choices=['upstream-simulation','hardware-control','recordings-only'],default='upstream-simulation')
    parser.add_argument('--gateway-url',default='ws://127.0.0.1:8776')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--media-url',default='http://127.0.0.1:8777')
    parser.add_argument('--episodes-dir',type=Path,default=Path(__file__).resolve().parents[2]/'runtime/episodes')
    parser.add_argument('--managed-stdin',action='store_true',help='Owned launcher pipe EOF requests normal app shutdown')
    args=parser.parse_args()
    from phosphobot.app import app,start_server
    from phosphobot.types import SimulationMode
    if args.backend=='upstream-simulation':install_simulation(app)
    else:
        from .pc_bridge import install_hardware
        install_hardware(app,args.gateway_url,media_url=args.media_url,episodes_dir=args.episodes_dir,
                         recordings_only=args.backend=='recordings-only')
    if args.managed_stdin:
        from .managed_app import install_parent_shutdown
        install_parent_shutdown(app)
    start_server(host='127.0.0.1',port=args.port,simulation=SimulationMode.headless,
        only_simulation=True,simulate_cameras=args.backend=='upstream-simulation',realsense=False,can=False,
        cameras=args.backend=='upstream-simulation',max_opencv_index=0,telemetry=False)


if __name__=='__main__':
    main()
