"""Run: PYTHONPATH=device_gateway:camera_tools python3 -m e6_observer."""
import argparse
import hashlib
import zstandard
import json
import signal
import sys
import threading
import time
from websockets.sync.server import serve
from .runtime import Hub
from .protocol import encode_wire
from .wifi_latency import WifiLatency
from .ssh_tcp_latency import SshTcpLatency
from .tcp_latency import TcpLatency,inspect_e6_route


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--robot-ip',default='192.168.1.4')
    p.add_argument('--camera-tools',default='/root/remaining-issues-20261002_2226/tools')
    p.add_argument('--instance',default='manual')
    p.add_argument('--parent-stdin',action='store_true')
    p.add_argument('--media-hz',type=int,choices=(5,10,20))
    p.add_argument('--wifi-low-latency',action='store_true')
    p.add_argument('--wifi-reorder-ms',type=int,choices=(10,),help='Temporary AIC8800 wlan0 reorder timeout; restored on exit')
    p.add_argument('--ssh-rto-min-ms',type=int,choices=(20,),help='Temporary host-route minimum RTO to this SSH peer; restored on exit')
    p.add_argument('--e6-rto-min-ms',type=int,choices=(20,),help='Temporary E6 host-route minimum RTO; owned control only, restored on exit')
    p.add_argument('--control-stage',choices=('shadow','live'))
    p.add_argument('--control-role',choices=('harness','operator'),default='harness')
    p.add_argument('--startup-config-stdin',action='store_true')
    a=p.parse_args()
    if a.e6_rto_min_ms is not None and not a.control_stage:p.error('E6 minimum RTO requires owned control gateway')
    if a.media_hz is None:a.media_hz=10 if a.control_stage else 20
    startup=json.loads(sys.stdin.buffer.readline()) if a.startup_config_stdin else {}
    if a.control_stage and not startup.get('control_token'):p.error('Control requires an owned startup session token')
    # Operator rollout is deliberately locked until hardware acceptance is implemented and verified.
    if a.control_role=='operator' and a.control_stage=='live' and not startup.get('operator_verified'):p.error('Live operator control has no verified acceptance profile')
    if startup.get('kinematics_profile') and not a.control_stage:p.error('Local kinematics are only available in a control session')
    hub=Hub(a.robot_ip,a.camera_tools,a.instance,a.control_stage,a.control_role,startup.get('control_token'),startup.get('control_limits'),startup.get('kinematics_profile'),startup.get('feedback_framing','strict'),startup.get('recovery_policy','manual'),startup.get('gripper_command_profile','plan32'))
    def archive(ws):
        try:
            while not hub.stop.is_set():
                command=json.loads(ws.recv())
                if command.get('instance')!=a.instance:raise ValueError('Wrong gateway instance')
                operation=command.get('operation');session=command.get('session')
                if operation=='begin':result=hub.archive.begin(session)
                elif operation=='seal':result=hub.archive.seal(session)
                elif operation=='status':result=hub.archive.status()
                elif operation=='release':result=hub.archive.release(session)
                elif operation=='fetch':
                    entry,data=hub.archive.fetch(session,command.get('source'),command.get('id'))
                    payload=zstandard.ZstdCompressor(level=1,write_checksum=True).compress(data)
                    ws.send(json.dumps(dict(ok=True,entry=entry,wire_bytes=len(payload),wire_sha256=hashlib.sha256(payload).hexdigest())))
                    # Pace small chunks; archival reliability must not starve realtime.
                    deadline=time.perf_counter()
                    for offset in range(0,len(payload),8192):
                        chunk=payload[offset:offset+8192];ws.send(chunk)
                        deadline+=len(chunk)/1_000_000
                        hub.stop.wait(max(0,deadline-time.perf_counter()))
                    continue
                else:raise ValueError('Only native media archive operations accepted')
                ws.send(json.dumps(dict(ok=True,archive=result)))
        except Exception as exc:
            try:ws.send(json.dumps(dict(ok=False,error=str(exc))))
            except Exception:pass
            ws.close(1008,'Archive operation failed')

    def handle(ws):
        if ws.request.path=='/ws/archive':archive(ws);return
        if ws.request.path not in ('/ws/observations','/ws/media'):
            ws.close(1008,'Observation gateway has no control endpoint'); return
        media_stream=ws.request.path=='/ws/media'
        period=1/a.media_hz if media_stream else .05
        seq=0;deadline=time.perf_counter();media_cache={}
        metrics={}
        sampling_owner=object() if a.control_stage and not media_stream else None
        sampling_acquired=False
        try:
            if sampling_owner is not None:
                hub.begin_control_sampling(sampling_owner);sampling_acquired=True
            while not hub.stop.is_set():
                try:
                    probe=json.loads(ws.recv(timeout=0))
                    if probe.get('operation')!='clock_probe' or not isinstance(probe.get('nonce'),str):
                        ws.close(1008,'Only observation clock probes are accepted');return
                    ws.send(json.dumps(dict(clock_probe=probe['nonce'],x5_monotonic_ns=time.perf_counter_ns())))
                except TimeoutError:pass
                if media_stream and time.perf_counter()-deadline>period:
                    with hub.lock:hub.stats['preview_discarded']+=int((time.perf_counter()-deadline)/period)
                    deadline=time.perf_counter()
                if time.perf_counter()-deadline>.25:
                    print(json.dumps(dict(publisher_late_s=time.perf_counter()-deadline,metrics=metrics)),flush=True)
                    ws.send(json.dumps(dict(error='Observation publisher overflow; restart explicitly: '+str(metrics))))
                    break
                begin=time.perf_counter()
                # Independent small telemetry packets keep robot feedback fresh
                # while native RGB-D uses a separately budgeted media channel.
                frame=hub.snapshot(seq,control_sample=not media_stream)
                if not media_stream:hub.archive.capture(frame)
                for name,key in (('rgb','jpeg'),('depth','uint16_raw')):frame['sources'][name].pop(key,None)
                if not media_stream:
                    for name in ('rgb','depth'):frame['sources'][name].pop('preview_jpeg',None)
                seq+=1
                frame['stats']['publisher']=metrics
                # Video bytes and duplicate diagnostics never travel over the control connection.
                frame['stats']['sampling']=dict(observation_hz=20,rgb_selection_hz=20,depth_selection_hz=20,preview_hz=a.media_hz,
                    preview_rgb_size=list(hub.preview_profile['rgb']),preview_depth_size=list(hub.preview_profile['depth']),preview_quality=hub.preview_profile['jpeg_quality'])
                wire=encode_wire(frame,media_cache);encoded=time.perf_counter()
                ws.send(wire)
                metrics=dict(encode_ms=(encoded-begin)*1000,send_ms=(time.perf_counter()-encoded)*1000,wire_bytes=len(wire),frame=seq)
                if media_stream:
                    with hub.lock:hub.stats['media_publisher']=metrics
                if a.control_stage and not media_stream:
                    # The guardian is the sole 20 Hz clock. A second pacing
                    # clock would accumulate jitter while waiting for its ticks.
                    deadline=time.perf_counter()
                else:
                    deadline+=period
                    hub.stop.wait(max(0,deadline-time.perf_counter()))
        except Exception: pass
        finally:
            if sampling_acquired:hub.end_control_sampling(sampling_owner)
    def request(connection,request):
        if request.path=='/health':
            return connection.respond(200,json.dumps(dict(backend='hardware-control' if a.control_stage else 'hardware-observe',instance=a.instance,capabilities={'control':bool(a.control_stage)},can_tx_count=0)))
        if request.path not in ('/ws/observations','/ws/media','/ws/archive'):return connection.respond(403,'Read-only gateway: endpoint rejected')
    with WifiLatency(a.wifi_low_latency,a.wifi_reorder_ms) as wifi, SshTcpLatency(a.ssh_rto_min_ms) as tcp, TcpLatency(a.robot_ip,a.e6_rto_min_ms) as e6_tcp:
        hub.stats['wifi_latency']=wifi.report
        hub.stats['ssh_tcp_latency']=tcp.report
        hub.stats['e6_tcp_latency']=e6_tcp.report
        hub.stats['e6_network']=inspect_e6_route(a.robot_ip)
        try:
            hub.start()
            with serve(handle,'127.0.0.1',8767,process_request=request,max_size=1024,compression=None) as server:
                def stop(*_):
                    hub.stop.set(); threading.Thread(target=server.shutdown,daemon=True).start()
                for sig in (signal.SIGINT,signal.SIGTERM): signal.signal(sig,stop)
                if a.parent_stdin:
                    def parent(): sys.stdin.buffer.read(); stop()
                    threading.Thread(target=parent,daemon=True).start()
                try: server.serve_forever()
                finally: hub.stop.set()
        finally:hub.close()


if __name__=='__main__':main()
