"""Cut exactly the owned control SSH socket during mapped zero increments.

Observation SSH stays open. This never enables, replays or changes E6/CAN
configuration. The caller supplies the socket endpoint and sshd PID recorded
by the current Tunnel; both are verified immediately before the cut.
"""
import argparse,asyncio,hashlib,ipaddress,json,os,re,shlex,time
from pathlib import Path
import numpy as np
import paramiko
import websockets
from live_signal_acceptance import LiveSignals
from control_acceptance import source_hashes


def cut_owned_socket(args):
    peer=str(ipaddress.IPv4Address(args.peer))
    if not ipaddress.ip_address(peer).is_private or not 1<=args.peer_port<=65535 or args.sshd_pid<=1:
        raise ValueError('Recorded private endpoint and sshd owner required')
    client=paramiko.SSHClient();client.load_system_host_keys();client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(args.host,username=args.user,password=os.environ.get('E6_X5_PASSWORD'),timeout=8)
    def command(arguments,details=False):
        _,out,err=client.exec_command(shlex.join(arguments));data=out.read().decode();status=out.channel.recv_exit_status()
        error=err.read().decode()
        if status:raise RuntimeError(error)
        return dict(stdout=data,stderr=error,exit_code=status) if details else data
    try:
        if client.get_transport().sock.getsockname()==(peer,args.peer_port):raise ValueError('Refuse to cut diagnostic SSH')
        filters=['dst',peer,'dport','=',':'+str(args.peer_port),'sport','=',':22']
        before=command(['ss','-Htnp','state','established',*filters]).splitlines()
        if len(before)!=1 or ('pid='+str(args.sshd_pid)+',') not in before[0] or '"sshd"' not in before[0]:
            raise ValueError('Control SSH identity changed; refuse socket cut')
        sent=time.perf_counter_ns()
        output=command(['ss','-Ktn',*filters],details=True)
        after=command(['ss','-Htnp','state','established',*filters]).strip()
        method='socket-destroy'
        if after:
            # Some vendor kernels reject SOCK_DESTROY while ss still exits0.
            # Recheck ownership, then terminate this session child only.
            rows=after.splitlines()
            if len(rows)!=1 or ('pid='+str(args.sshd_pid)+',') not in rows[0] or '"sshd"' not in rows[0]:
                raise RuntimeError('Control SSH identity changed after socket destroy; refuse termination')
            process=command(['ps','-p',str(args.sshd_pid),'-o','args=']).strip()
            if not re.fullmatch(r'sshd: [^\s]+@notty',process):
                raise RuntimeError('Refuse termination of a non-session sshd process')
            command(['kill','-TERM',str(args.sshd_pid)])
            method='exact-owned-session-termination'
            deadline=time.monotonic()+1
            while after and time.monotonic()<deadline:
                time.sleep(.05)
                after=command(['ss','-Htnp','state','established',*filters]).strip()
            if after:raise RuntimeError('Owned control SSH termination not confirmed')
        return dict(kind='control-ssh',pc_injection_requested_ns=sent,pc_injection_confirmed_ns=time.perf_counter_ns(),
                    peer=peer,peer_port=args.peer_port,sshd_pid=args.sshd_pid,before=before,output=output,after=after,
                    method=method,scope='exact owned control SSH TCP session; no network rules or other sessions changed')
    finally:client.close()


async def run(args,runner):
    runner.start_ns=time.perf_counter_ns();observer=asyncio.create_task(runner.observe());failure=None;injection=None
    ws=None;confirmed=False
    try:
        deadline=time.monotonic()+15
        while runner.first is None:
            if observer.done():await observer
            if time.monotonic()>deadline:raise TimeoutError('Observation not ready')
            await asyncio.sleep(.05)
        first=runner.latest
        if first['backend']!='hardware-control' or first['control_stage']!='live' or runner.actual()['robot_mode']!=5 or runner.actual()['enable_status']!=1:
            raise ValueError('Already enabled idle live session required')
        if first['stats'].get('e6_network')!=runner.shadow.get('e6_network'):raise ValueError('Current E6 link differs from shadow acceptance')
        if first['stats'].get('sampling',{}).get('preview_hz')!=runner.shadow.get('media_hz'):raise ValueError('Current preview rate differs from shadow acceptance')
        from e6_teleop.pc_tcp_latency import configuration
        if configuration(first['stats'].get('pc_tcp_latency') or {})!=runner.shadow.get('pc_tcp_configuration'):
            raise ValueError('Current PC TCP configuration differs from shadow acceptance')
        expected=runner.shadow['final_status']
        if first['status']['limits']!=expected['limits'] or first['status']['kinematics']['profile_sha256']!=expected['kinematics']['profile_sha256']:
            raise ValueError('Current configuration differs from shadow acceptance')
        for source,fields in [('wifi_latency',('requested','driver_ps_on','reorder_requested_ms','reorder_effective_ms')),
                              ('ssh_tcp_latency',('requested_min_rto_ms','applied','peer','source')),
                              ('e6_tcp_latency',('requested_ms','changed','applied_route'))]:
            if any(first['stats'][source].get(k)!=runner.shadow[source].get(k) for k in fields):
                raise ValueError('Current transport differs from shadow acceptance')
        token=os.environ['E6_TELEOP_HARNESS_TOKEN']
        ws=await websockets.connect('ws://127.0.0.1:8765/ws/control',proxy=None,additional_headers={'X-E6-Control-Token':token})
        runner.ws=ws;hello=json.loads(await ws.recv())
        if not hello.get('ok'):raise RuntimeError(str(hello))
        runner.status=hello['status'];runner.receiver=asyncio.create_task(runner.replies());runner.fresh_input_session=True
        await runner.rpc(dict(operation='stop'))
        n=runner.neutral('keyboard');await runner.phase(5.5,n);await runner.prepare('keyboard')
        q0=np.asarray(runner.actual()['joints_rad']);await runner.phase(.1,dict(n,deadman=True),True)
        before_tick=runner.latest['control']['tick']['control_tick_id']
        injection=await asyncio.to_thread(cut_owned_socket,args)
        # No heartbeat, reprepare, reconnect or action follows the injected cut.
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            if observer.done():await observer
            state=runner.latest.get('status') or {}
            if not state.get('armed') and state.get('stop_acknowledged') and state.get('stopped_feedback'):
                confirmed=True;break
            await asyncio.sleep(.025)
        if not confirmed:raise RuntimeError('SSH cut did not confirm Stop and stationary feedback')
        await asyncio.sleep(.35)
        state=runner.latest['status'];q1=np.asarray(runner.actual()['joints_rad'])
        if state.get('armed') or float(np.max(np.abs(q1-q0)))>np.radians(.05):raise ValueError('Cut resumed or moved outside zero-increment tolerance')
        if runner.invalid or source_hashes()!=runner.source_manifest:raise ValueError('Invalid sources or changed implementation')
        runner.checks.append(dict(case='control-ssh',passed=True,explicit_reprepare_required=True,automatic_reconnect=False,
                                 stop_acknowledged=True,stopped_feedback=True,last_pre_injection_tick=before_tick,
                                 max_joint_change_rad=float(np.max(np.abs(q1-q0))),status_reason=state.get('reason')))
        runner.completed=True
    except BaseException as exc:
        failure=str(exc);raise
    finally:
        if not confirmed and ws and ws.state.name!='CLOSED':
            try:await runner.rpc(dict(operation='stop'))
            except Exception:pass
        observer.cancel();await asyncio.gather(observer,return_exceptions=True)
        if runner.pending_evidence:await runner.pending_evidence
        runner.stream.close()
        if runner.receiver:runner.receiver.cancel();await asyncio.gather(runner.receiver,return_exceptions=True)
        if ws:await ws.close()
        summary=dict(passed=runner.completed,failure=failure,suite='control-ssh-fault',stage='live',frames=runner.frames,
                     invalid=runner.invalid,action_ticks=runner.action_ticks,candidate_ticks=runner.candidate_ticks,
                     checks=runner.checks,injection=injection,duration_s=(time.perf_counter_ns()-runner.start_ns)/1e9,
                     source_sha256=runner.source_manifest,harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                     final_status=runner.latest and runner.latest.get('status'),synthetic_inputs_only=True,
                     scope='physical zero-increment E6 and exact control SSH socket; observation stream intact')
        # Status capabilities are session secrets, even after the cut.
        if summary['final_status']:
            summary['final_status']={k:v for k,v in summary['final_status'].items() if k not in ('session','lease')}
        (args.evidence/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in summary.items() if k not in ('source_sha256','final_status')},ensure_ascii=True),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--host',default='192.168.1.13');p.add_argument('--user',default='root')
    p.add_argument('--peer',required=True);p.add_argument('--peer-port',type=int,required=True);p.add_argument('--sshd-pid',type=int,required=True)
    p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--shadow-evidence',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args();runner=LiveSignals(json.loads(a.fixtures.read_text(encoding='utf-8')),a.evidence,a.shadow_evidence)
    asyncio.run(run(a,runner))
