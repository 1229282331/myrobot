"""Local-only full phosphobot recording recovery fixture. No device gateway.

Use a retained immutable bundle, a fresh output directory, and loopback media.
Create `restore` in output to remove the owned status-write obstruction; then
click the real page retry button. Create `stop` for normal app shutdown.
"""
import argparse
import json
from pathlib import Path
import shutil
import signal
import threading
import time
from types import SimpleNamespace

from e6_phosphobot.camera_archive import atomic_json
from e6_phosphobot.episode_http import start_server
from e6_phosphobot.episode_transfer import digest_file
from e6_phosphobot.episodes import Episodes,checked_id


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=18765)
    parser.add_argument('--media-port',type=int,default=18777)
    args=parser.parse_args()
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    episode=checked_id(args.source.name)
    remote=args.output/'remote'/episode;remote.mkdir(parents=True)
    for name in ('bundle.zip','summary.json'):shutil.copyfile(args.source/name,remote/name)
    digest=digest_file(remote/'bundle.zip')
    atomic_json(remote/'transfer.json',dict(episode=episode,bundle_sha256=digest,phase='pending'))
    episodes=Episodes(args.output/'remote',args.output/'no-camera',SimpleNamespace(),'shadow')
    episodes.value.update(phase='transfer_pending',episode=episode,bundle_sha256=digest,result='success')
    pc=args.output/'pc'/episode;pc.mkdir(parents=True)
    obstruction=pc/'pc_transfer.tmp';obstruction.mkdir()
    marker=obstruction/'fixture-owner.json'
    ownership=dict(fixture='local-transfer',output=str(args.output),episode=episode)
    atomic_json(marker,ownership)
    server=start_server(episodes,args.media_port,bytes_per_second=8*1024*1024)
    from phosphobot.app import app,start_server as start_app
    from phosphobot.types import SimulationMode
    from e6_phosphobot.pc_bridge import install_hardware
    cache=install_hardware(app,'ws://127.0.0.1:18776',
        media_url=f'http://127.0.0.1:{args.media_port}',episodes_dir=args.output/'pc')
    caps=cache.capabilities();caps['backend']='本地回传故障测试 · 无实机连接'
    cache.value=dict(capabilities=caps)
    requests=[]
    @app.middleware('http')
    async def record_requests(request,call_next):
        if request.method=='POST':requests.append(dict(path=request.url.path,at_ns=time.perf_counter_ns()))
        return await call_next(request)
    halt=threading.Event()
    def supervise():
        while not halt.wait(.1):
            if (args.output/'restore').exists() and obstruction.exists():
                assert obstruction.resolve().parent==pc.resolve()
                assert json.loads(marker.read_text(encoding='utf8'))==ownership
                marker.unlink();obstruction.rmdir()
                atomic_json(args.output/'restored.json',dict(owned_obstruction_removed=True,at_ns=time.perf_counter_ns()))
            if (args.output/'stop').exists():signal.raise_signal(signal.SIGINT);return
    monitor=threading.Thread(target=supervise,daemon=True);monitor.start()
    atomic_json(args.output/'fixture.json',dict(fixture_only=True,physical_connections=0,
        source_bundle=str(args.source/'bundle.zip'),bundle_sha256=digest,url=f'http://127.0.0.1:{args.port}'))
    try:
        start_app(host='127.0.0.1',port=args.port,simulation=SimulationMode.headless,
            only_simulation=True,simulate_cameras=False,realsense=False,can=False,cameras=False,
            max_opencv_index=0,telemetry=False)
    finally:
        halt.set();monitor.join(2);server.shutdown();server.server_close();episodes.close()
        atomic_json(args.output/'result.json',dict(fixture_only=True,physical_connections=0,
            remote=episodes.state(),post_requests=requests,obstruction_remaining=obstruction.exists()))


if __name__=='__main__':
    import typer
    try:main()
    except typer.Exit as exc:raise SystemExit(exc.exit_code)
