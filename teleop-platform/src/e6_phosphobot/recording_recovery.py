"""One entry point to the existing media protocol and phosphobot record browser.

Only one SSH media tunnel and a file server are started. No gateway, camera,
MoveIt, CAN, MediaMTX, device observer, or robot-control websocket is started.
"""
import json
import shlex
import threading
import time
import uuid
from urllib.request import urlopen
import webbrowser

from .launcher import ROOT, LocalServices, Tunnels, deploy, ssh_factory


def recovery_command(deployed, identity, episode=None):
    # Reuse the installed gateway's media libraries (not system Python alone).
    # This is the same isolated import path used by the session supervisor.
    command=['env','PYTHONPATH='+deployed['release']+':'+deployed['gateway_dependencies'],
             'PYTHONDONTWRITEBYTECODE=1','python3','-B','-m','e6_phosphobot.episode_http',
             '--sessions-root',deployed['remote_base']+'/sessions','--managed-stdin','--recovery-id',identity]
    if episode is not None:command+=['--episode',episode]
    return command


def run(config, *, connect=None, open_browser=True, halt=None, duration=None, episode=None):
    from .episodes import checked_id
    if episode is not None:checked_id(episode)
    identity=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
    runtime=ROOT/'runtime/recoveries'/identity
    runtime.mkdir(parents=True)
    halted=halt or threading.Event()
    local=LocalServices(runtime,False)
    tunnels=None;channel=None;stdin=None;reader=None;log=None
    result=dict(recovery=identity,devices_started=False,ready=False,remote_started=False)
    try:
        tunnels=Tunnels(connect or ssh_factory(config),media_only=True)
        tunnels.start()
        deployed=deploy(tunnels.media,config,identity,runtime)
        command=recovery_command(deployed,identity,episode)
        stdin,stdout,stderr=tunnels.media.exec_command(shlex.join(command))
        channel=stdout.channel;channel.set_combine_stderr(True)
        result['remote_started']=True
        log=(runtime/'remote.log').open('w',encoding='utf8')
        def collect():
            for line in stdout:log.write(line);log.flush()
        reader=threading.Thread(target=collect,daemon=True);reader.start()
        deadline=time.monotonic()+15
        while not halted.is_set():
            if channel.exit_status_ready():raise RuntimeError('Retained media service exited; see remote.log')
            try:
                with urlopen('http://127.0.0.1:8777/episodes',timeout=2) as reply:listing=json.load(reply)
                if (not listing.get('current',{}).get('recovery_only') or
                        listing['current'].get('recovery_id')!=identity):
                    raise ValueError('Media port belongs to another session; close it before recovery')
                break
            except (OSError,TimeoutError):
                if time.monotonic()>deadline:raise TimeoutError('Retained media service did not become ready')
                halted.wait(.1)
        if halted.is_set():raise RuntimeError('Recovery cancelled before ready')
        python=ROOT/'upstream/phosphobot/phosphobot/.venv/Scripts/python.exe'
        local.ensure(8765,[str(python),'-B','-X','utf8',str(ROOT/'scripts/run_pc_owned.py'),
                          '--backend','recordings-only'],
                     [ROOT/'scripts/run_pc_owned.py','--backend','recordings-only'],app=True)
        result.update(ready=True,discovered=len(listing['episodes']),catalogue_errors=listing.get('catalogue_errors',[]))
        print('历史记录恢复已启动；只回传和浏览，不连接设备。Ctrl+C 退出。日志：'+str(runtime),flush=True)
        if open_browser:webbrowser.open('http://127.0.0.1:8765/control?panel=episodes')
        begin=time.monotonic()
        while not halted.wait(.2):
            if channel.exit_status_ready():raise RuntimeError('Retained media service disconnected')
            if duration is not None and time.monotonic()-begin>=duration:halted.set()
    except Exception as exc:
        result['error']=str(exc)
        print('恢复未完成：'+str(exc),flush=True)
    finally:
        # Stop the owned TransferWorker before closing its media transport.
        result['local_cleanup_errors']=local.close()
        result['local_exits']=local.exits
        if channel:
            if not channel.exit_status_ready():
                try:stdin.write('stop\n');stdin.flush();channel.shutdown_write()
                except (OSError,EOFError):pass
            deadline=time.monotonic()+15
            while not channel.exit_status_ready() and time.monotonic()<deadline:time.sleep(.1)
            if channel.exit_status_ready():result['remote_exit']=channel.recv_exit_status()
            else:result['remote_cleanup_uncertain']=True
        if tunnels:tunnels.close()
        if reader:reader.join(3)
        if reader and reader.is_alive():result['log_reader_pending']=True
        elif log:log.close()
        result['clean']=(not result.get('error') and not result['local_cleanup_errors'] and
                         not result.get('log_reader_pending') and
                         (not result['remote_started'] or result.get('remote_exit')==0))
        (runtime/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
        print(json.dumps(result,ensure_ascii=False),flush=True)
    return result
