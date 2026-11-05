"""Run both simulation processes; all listeners are bound to loopback."""
import argparse
import json
import socket
import os
import secrets
from pathlib import Path
import subprocess
import sys
import time
import urllib.request
import uvicorn
from .model import ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["serve", "gateway", "export"], nargs="?", default="serve")
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--episode", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--backend",choices=["simulation","hardware-observe","hardware-control"],default="simulation")
    parser.add_argument('--control-stage',choices=('shadow','live'),default='shadow')
    parser.add_argument('--controller-role',choices=('harness','operator'),default='harness')
    parser.add_argument('--harness-limits',choices=('initial','delivery'),default='initial',help='Startup-fixed automatic-test caps; delivery is 5 mm/s, 2 deg/s, 10 percent/s. Operator still needs acceptance.')
    parser.add_argument('--acceptance-profile',type=Path,help='Verified source/device/evidence profile required for physical operator rollout')
    parser.add_argument('--gripper-profile',choices=('plan32','empty64-medium','disabled'),default='plan32',help='Startup-fixed CAN profile; disabled prohibits CAN initialization and transmission')
    parser.add_argument('--kinematics',choices=('native','controller-local'),default='native',help='Startup-fixed control kinematics; observe remains read-only')
    parser.add_argument('--kinematics-profile',type=Path,help='Immutable controller sampling bundle for controller-local')
    parser.add_argument('--feedback-framing',choices=('strict','length-envelope'),default='strict',help='Opt-in observed uint64 length envelope; SDK payload checks remain unchanged')
    parser.add_argument('--recovery-policy',choices=('manual','bounded'),default='bounded',help='Transient faults can automatically prepare after confirmed Stop, 5 s healthy neutral input; never replay targets')
    parser.add_argument('--pc-rto-min-ms',type=int,choices=(20,),help='Validate an explicitly configured Linux PC peer route; never modifies Windows TCP settings')
    parser.add_argument("--ssh-host",help="Existing trusted X5 SSH host; optional for an external tunnel")
    parser.add_argument("--ssh-user",default="root")
    parser.add_argument('--ssh-runtime',choices=('native','wsl'),default='native',help='Windows PC service can delegate only SSH TCP sockets to existing trusted WSL runtime')
    parser.add_argument("--remote-root",help="Explicitly start our deployed observer here; stdin EOF releases its resources")
    parser.add_argument('--wifi-low-latency',action='store_true',help='Temporary iw power-save request; firmware effect is driver-dependent and unverified')
    parser.add_argument('--wifi-reorder-ms',type=int,choices=(10,),help='Temporary X5 AIC8800 wlan0 reorder timeout; requires owned SSH gateway')
    parser.add_argument('--ssh-rto-min-ms',type=int,choices=(20,),help='Temporary X5 host-route minimum TCP RTO to this PC; restored on exit')
    parser.add_argument('--e6-rto-min-ms',type=int,choices=(20,),help='Temporary X5 minimum TCP RTO to E6; owned control only, restored on exit')
    parser.add_argument('--media-hz',type=int,choices=(5,10,20),help='Preview rate (control default 10 Hz, observe 20 Hz); native sampling remains 20 Hz')
    args = parser.parse_args()
    if args.ssh_runtime=='wsl' and (sys.platform!='win32' or args.backend!='hardware-control' or args.pc_rto_min_ms!=20):
        parser.error('WSL SSH runtime requires Windows hardware-control and explicit PC peer RTO 20 ms')
    if args.pc_rto_min_ms is not None and (args.backend!='hardware-control' or not args.ssh_host or not args.remote_root):parser.error('--pc-rto-min-ms requires owned hardware-control SSH gateway')
    if args.wifi_reorder_ms is not None and (args.backend=='simulation' or not args.ssh_host or not args.remote_root):parser.error('--wifi-reorder-ms requires owned hardware SSH gateway')
    if args.ssh_rto_min_ms is not None and (args.backend!='hardware-control' or not args.ssh_host or not args.remote_root):parser.error('--ssh-rto-min-ms requires owned hardware-control SSH gateway')
    if args.e6_rto_min_ms is not None and (args.backend!='hardware-control' or not args.ssh_host or not args.remote_root):parser.error('--e6-rto-min-ms requires owned hardware-control SSH gateway')
    if args.harness_limits!='initial' and (args.backend!='hardware-control' or args.controller_role!='harness'):parser.error('--harness-limits delivery requires hardware-control with harness role')
    if (args.kinematics=='controller-local') != bool(args.kinematics_profile):parser.error('controller-local requires --kinematics-profile; native accepts no profile')
    if args.kinematics_profile and args.backend!='hardware-control':parser.error('Local calibration requires hardware-control')
    if args.feedback_framing!='strict' and args.backend!='hardware-control':parser.error('Length-envelope validation is only available in control acceptance sessions')
    if args.gripper_profile!='plan32' and args.backend!='hardware-control':parser.error('Gripper command profile requires hardware-control')
    if args.command == "gateway":
        uvicorn.run("e6_teleop.gateway:app", host="127.0.0.1", port=8766, log_level="warning")
    elif args.command == "export":
        if not args.episode or not args.output: parser.error("export requires --episode and --output")
        from .exporter import export_episode
        export_episode(args.episode, args.output)
    else:
        if not (ROOT / "web" / "dist" / "index.html").exists():
            parser.error("Build the web frontend first: cd teleop/web; npm ci; npm run build")
        if args.backend in ('hardware-observe','hardware-control'):
            with socket.socket() as probe:
                if sys.platform=='linux':probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                try:probe.bind(('127.0.0.1',8765))
                except OSError:parser.error('Loopback port 8765 is already in use; close the existing workbench first')
            from .observation_service import make_app
            from .tunnel import Tunnel
            if args.ssh_runtime=='wsl':
                from .wsl_tunnel import WslTunnel
                Tunnel=WslTunnel
            tunnel=None
            try:
                if args.remote_root and not args.ssh_host:parser.error('--remote-root requires --ssh-host')
                controlling=args.backend=='hardware-control'
                if controlling and (not args.ssh_host or not args.remote_root):parser.error('hardware-control requires owned X5 --ssh-host and --remote-root')
                profile=None
                if controlling and args.controller_role=='operator':
                    if not args.acceptance_profile:parser.error('Operator control requires a complete hardware acceptance profile')
                    from .acceptance_profile import verify_profile
                    try:profile=verify_profile(args.acceptance_profile)
                    except (ValueError,OSError,KeyError) as exc:parser.error(str(exc))
                    expected_kind='controller-local' if args.kinematics=='controller-local' else 'controller-native'
                    if profile.get('kinematics_kind','controller-native')!=expected_kind or profile.get('feedback_framing','strict')!=args.feedback_framing:parser.error('Startup kinematics/framing differ from hardware acceptance')
                token=os.environ.get('E6_TELEOP_HARNESS_TOKEN') or secrets.token_urlsafe(32) if controlling else None
                if args.ssh_host:tunnel=Tunnel(args.ssh_host,args.ssh_user,args.remote_root,wifi_low_latency=args.wifi_low_latency,media_hz=args.media_hz,control_stage=args.control_stage if controlling else None,control_role=args.controller_role,control_token=token,acceptance_profile=profile,kinematics_profile=args.kinematics_profile,feedback_framing=args.feedback_framing,harness_limits=args.harness_limits,wifi_reorder_ms=args.wifi_reorder_ms,ssh_rto_min_ms=args.ssh_rto_min_ms,e6_rto_min_ms=args.e6_rto_min_ms,pc_rto_min_ms=args.pc_rto_min_ms,recovery_policy=args.recovery_policy if controlling else 'manual',gripper_command_profile=args.gripper_profile).start()
                print(f'E6 {args.backend} workbench: http://127.0.0.1:8765',flush=True)
                app=make_app(args.data_root,archive_gateway=f'ws://127.0.0.1:{tunnel.archive_port}' if tunnel else None,backend=args.backend,
                                    gateway=f'ws://127.0.0.1:{tunnel.telemetry_port}' if controlling else 'ws://127.0.0.1:8767',media_gateway='ws://127.0.0.1:8767',
                                    control_gateway=f'ws://127.0.0.1:{tunnel.control_port}' if controlling else None,control_token=token,control_role=args.controller_role,pc_tcp_profile=tunnel.pc_tcp_profile if tunnel else None,
                                    exit_token=os.environ.get('E6_TELEOP_EXIT_TOKEN') or token,exit_callback=lambda:setattr(server,'should_exit',True))
                server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=8765,log_level='warning'))
                server.run()
            finally:
                if tunnel:tunnel.close()
            return
        for port in (8765, 8766):
            with socket.socket() as probe:
                try: probe.bind(("127.0.0.1", port))
                except OSError: parser.error(f"Loopback port {port} is already in use; close the existing workbench first")
        flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
        instance = secrets.token_urlsafe(24)
        child = subprocess.Popen([sys.executable, "-m", "e6_teleop", "gateway"], creationflags=flags,
                                 env={**os.environ, "E6_SIM_INSTANCE": instance})
        try:
            for _ in range(150):
                if child.poll() is not None: raise RuntimeError("Simulation gateway failed to start")
                try:
                    with urllib.request.urlopen("http://127.0.0.1:8766/health", timeout=1) as response:
                        health = json.load(response)
                    if health.get("instance") != instance:
                        raise RuntimeError("Unexpected process owns simulation gateway port")
                    break
                except Exception: time.sleep(.1)
            else: raise RuntimeError("Simulation gateway startup timeout")
            from .service import make_app
            print("E6 simulation workbench: http://127.0.0.1:8765 (no hardware connections)", flush=True)
            uvicorn.run(make_app(args.data_root), host="127.0.0.1", port=8765, log_level="warning")
        finally:
            if sys.platform == "win32":
                # Windows venv launchers spawn the interpreter: terminate our entire child tree.
                subprocess.run(["taskkill", "/PID", str(child.pid), "/T", "/F"], capture_output=True)
            else: child.terminate()
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: child.kill(); child.wait()


if __name__ == "__main__": main()
