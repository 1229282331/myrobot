"""Live zero-increment readiness recovery only; no enable, CAN or joint goals."""
import argparse,asyncio,hashlib,json,os,time
from pathlib import Path
import websockets
from control_acceptance import Harness,source_hashes,require_shadow_acceptance


async def run(fixtures,signals,destination):
    h=Harness(fixtures,destination);observer=asyncio.create_task(h.observe());passed=False;reason=None
    start=time.perf_counter_ns();before=None;after=None;cause=None
    try:
        until=time.monotonic()+8
        while not h.latest or not h.latest.get('observation',{}).get('valid'):
            if time.monotonic()>until:raise RuntimeError('Live observations not ready')
            await asyncio.sleep(.1)
        if h.latest.get('control_stage')!='live':raise ValueError('Owned live harness required')
        d=h.latest['sources']['robot']['decoded']
        if d['robot_mode']!=5 or d['enable_status']!=1 or d['error_status'] or d['collision_state']:
            raise ValueError('Requires an already enabled, normal, stopped robot; no enable or recovery write performed')
        async with websockets.connect('ws://127.0.0.1:8765/ws/control',additional_headers={'X-E6-Control-Token':os.environ['E6_TELEOP_HARNESS_TOKEN']},proxy=None) as ws:
            h.ws=ws;hello=json.loads(await ws.recv())
            if not hello.get('ok'):raise RuntimeError('Control owner unavailable')
            h.status=hello['status'];h.receiver=asyncio.create_task(h.replies())
            if h.status.get('recovery',{}).get('policy')!='bounded':raise ValueError('Bounded recovery required')
            if h.status.get('gripper_initialized'):raise ValueError('This test must not have an active CAN controller')
            n=h.neutral();zero=signals['signals'][n['source']];h.fresh_input_session=True
            if zero['identity']!=n['identity'] or not zero['deadman'] or any(zero['twist']) or zero['gripper_rate']:
                raise ValueError('Fresh mapped zero signal required')
            await h.phase(5.6,n);await h.prepare(n)
            before=h.latest['sources']['robot']['decoded']['q_actual_deg']
            await h.phase(.1,zero,True)
            await asyncio.sleep(.4)  # Deliberate sender-only lease expiration.
            until=time.monotonic()+2
            while h.status['armed']:
                if time.monotonic()>until:raise RuntimeError('Input lease did not stop control')
                await asyncio.sleep(.02)
            cause=h.status.get('first_fault')
            if not cause or cause['reason']!='输入看门狗超时':raise RuntimeError('Unexpected fault instead of lease injection')
            await h.phase(6.7,n)
            if not h.status['armed'] or h.status['recovery']['state']!='ready':raise RuntimeError('Live automatic preparation did not complete')
            if h.status.get('stop_acknowledged') is not True or h.status.get('stopped_feedback') is not True:
                raise RuntimeError('Real Stop and stationarity were not confirmed')
            if h.status['recovery']['total_attempts']!=1:raise RuntimeError('Unexpected retry count')
            after=h.latest['sources']['robot']['decoded']['q_actual_deg']
            if max(abs(a-b) for a,b in zip(before,after))>.03:raise RuntimeError('Zero-increment hold exceeded .03 degrees')
            if h.invalid:raise RuntimeError('Invalid required observation')
            if h.actions<1:raise RuntimeError('No confirmed zero-increment action retained')
            if h.latest['control']['stats']['can_tx']!=0:raise RuntimeError('Unexpected CAN transmission')
            if source_hashes()!=h.source_manifest:raise RuntimeError('Source changed during live test')
            # Finish by releasing/disarming; do not test a nonzero move or replay.
            await h.rpc(dict(operation='stop'));await h.phase(1.2,n)
            if h.status.get('stop_acknowledged') is not True or h.status.get('stopped_feedback') is not True:
                raise RuntimeError('Final Stop not confirmed')
            passed=True
    except Exception as exc:reason=str(exc)
    finally:
        if h.ws and h.ws.state.name!='CLOSED' and h.status.get('stop_acknowledged') is not False:
            try:await h.rpc(dict(operation='stop'))
            except Exception:pass
        await asyncio.sleep(.3);observer.cancel();await asyncio.gather(observer,return_exceptions=True)
        if h.pending_evidence:await h.pending_evidence
        h.stream.close()
        if h.receiver:h.receiver.cancel();await asyncio.gather(h.receiver,return_exceptions=True)
        if h.ws:await h.ws.close()
        final={k:v for k,v in (h.status or {}).items() if k not in ('lease','session')}
        summary=dict(passed=passed,failure=reason,stage='live',suite='bounded-recovery-zero',duration_s=(time.perf_counter_ns()-start)/1e9,
            frames=h.frames,invalid=h.invalid,actions=h.actions,action_ticks=h.action_ticks,candidate_ticks=h.candidate_ticks,source_sha256=h.source_manifest,
            production_input_sha256=fixtures['production_input_sha256'],script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            actual_before_deg=before,actual_after_deg=after,injected_fault=cause,stats=(h.latest or {}).get('control',{}).get('stats'),final_status=final,
            old_action_replayed=False,enable_commands=0,can_commands=0,physical_motion_test=True,test_scope='zero increments and one input-lease recovery only',
            eligible_for_operator=False,hardware_sources_verified=bool(h.first and h.first['sources']['robot'].get('raw_base64')))
        (destination/'summary.json').write_bytes((json.dumps(summary,ensure_ascii=False,indent=2)+'\n').encode())
        print(json.dumps({k:summary[k] for k in ('passed','failure','frames','actions','actual_before_deg','actual_after_deg')},ensure_ascii=False))
    if not passed:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--signals',type=Path,required=True)
    p.add_argument('--shadow-evidence',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    fixtures=json.loads(a.fixtures.read_bytes());signals=json.loads(a.signals.read_bytes());require_shadow_acceptance(a.shadow_evidence,fixtures)
    if signals['production_input_sha256']!=fixtures['production_input_sha256'] or signals['profile']!=fixtures['profile']:raise ValueError('Zero signals do not match the passing production mapping/profile')
    if a.evidence.exists():raise ValueError('New evidence directory required')
    asyncio.run(run(fixtures,signals,a.evidence))
