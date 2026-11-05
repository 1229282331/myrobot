"""Qualification ONLY: production gateway + official SDK with socketpair devices.

No network device connection, CAN driver, camera, or ROS child can be created.
The UI explicitly reports a test fixture. This is not an E6 enable result.
"""
import argparse
import asyncio
import json
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
import numpy as np
from dobot_sdk.protocol.data_types import MyType,TEST_VALUE_MAGIC
from dobot_sdk.api.robot_control import RobotControl
from dobot_sdk.api.motion import Motion
from e6_phosphobot import gateway
from e6_phosphobot.dashboard import Dashboard,SubmissionPort
from e6_phosphobot.feedback import FeedbackReader


class Fixture(gateway.NativeExecutor):
    output=None
    def start(self):
        self.fixture_halt=threading.Event();self.fixture_enabled=False;self.fixture_commands=[]
        self.feedback_in,self.feedback_out=socket.socketpair()
        self.command_in,self.command_out=socket.socketpair();self.command_out.settimeout(.1)
        used=[False]
        def feedback_socket(*_args,**_kwargs):
            if used[0]:raise ConnectionError('Fixture connection ended; no device fallback')
            used[0]=True;return self.feedback_in
        self.reader=FeedbackReader('fixture-only',socket_factory=feedback_socket)
        self.dashboard=Dashboard('fixture-only');self.dashboard._socket=self.command_in
        self.dashboard._connected=True;self.dashboard._start_receiver()
        self.robot=RobotControl(SubmissionPort(self.dashboard));self.motion=Motion(SubmissionPort(self.dashboard))
        self.camera=lambda:dict(valid=True,fixture=True)
        def feedback():
            packet=np.zeros(1,dtype=MyType)
            packet['len']=1440;packet['TestValue']=TEST_VALUE_MAGIC
            packet['ToolVectorActual'][0]=[0,0,400,0,0,0]
            packet['ToolVectorTarget'][0]=[0,0,400,0,0,0]
            sequence=0
            while not self.fixture_halt.is_set():
                sequence+=1;packet['TimeStamp']=sequence;packet['RunTime']=sequence
                packet['RobotMode']=5 if self.fixture_enabled else 4
                packet['EnableStatus']=int(self.fixture_enabled)
                try:self.feedback_out.sendall(packet.tobytes())
                except OSError:break
                self.fixture_halt.wait(.008)
        def commands():
            buffer=b''
            while not self.fixture_halt.is_set():
                try:chunk=self.command_out.recv(4096)
                except socket.timeout:continue
                except OSError:break
                if not chunk:break
                buffer+=chunk
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1);command=line.decode('ascii')
                    self.fixture_commands.append(dict(command=command,at_ns=time.perf_counter_ns()))
                    accepted=command in ('EnableRobot()','Stop()')
                    if command=='EnableRobot()':self.fixture_enabled=True
                    try:self.command_out.sendall(f'{0 if accepted else -1},{{}},{command};'.encode())
                    except OSError:return
        self.fixture_threads=[threading.Thread(target=feedback,daemon=True),threading.Thread(target=commands,daemon=True)]
        self.reader.start()
        for thread in self.fixture_threads:thread.start()

    def _connect_dashboard(self):
        raise RuntimeError('Fixture has no network reconnect or physical device fallback')

    def state(self,controller=False):
        value=super().state(controller)
        value['fixture_only']=True;value['capabilities']['backend']='测试替身 · 无实机连接'
        value['capabilities']['video_url']=None;value['capabilities']['synthetic_keys']=False
        return value

    def close(self):
        try:super().close()
        finally:
            self.fixture_halt.set()
            for sock in (self.feedback_out,self.command_out):sock.close()
            for thread in self.fixture_threads:thread.join(1)
            self.output.write_text(json.dumps(dict(fixture_only=True,physical_commands=0,can_sends=0,
                commands=self.fixture_commands,enable_phase=self.enable_phase,
                enable_receipt=self.enable_receipt,threads_exited=all(not t.is_alive() for t in self.fixture_threads)),indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=18776);p.add_argument('--seconds',type=float,default=240)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);Fixture.output=a.output/'fixture.json'
    gateway.NativeExecutor=Fixture
    args=SimpleNamespace(host='fixture-only',stage='live',camera_health=None,acceptance_inputs=False,
        servo_period_s=.05,servo_aheadtime=50,servo_gain=500,release_mode='immediate',moveit_bundle=None,
        motion_profile='commissioning',audit_dir=a.output/'audit',episodes_dir=None,port=a.port)
    async def serve():
        task=asyncio.create_task(gateway.run(args))
        try:await asyncio.wait_for(asyncio.shield(task),a.seconds)
        except asyncio.TimeoutError:task.cancel();await asyncio.gather(task,return_exceptions=True)
    asyncio.run(serve())


if __name__=='__main__':main()
