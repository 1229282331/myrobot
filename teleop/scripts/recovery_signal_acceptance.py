"""Shadow-only lease-gap injection through the production mapped PC/X5 path."""
import argparse,asyncio,hashlib,json,os,time
from pathlib import Path
import websockets
from control_acceptance import Harness,source_hashes


async def run(fixtures,destination):
    h=Harness(fixtures,destination);observer=asyncio.create_task(h.observe())
    passed=False;reason=None;cases=[];started=time.perf_counter_ns()
    try:
        deadline=time.monotonic()+8
        while not h.latest or not h.latest.get('observation',{}).get('valid'):
            if time.monotonic()>deadline:raise RuntimeError('Healthy observations not ready')
            await asyncio.sleep(.1)
        if h.latest.get('control_stage')!='shadow':raise ValueError('Fault injection is shadow-only; no live device writes allowed')
        async with websockets.connect('ws://127.0.0.1:8765/ws/control',additional_headers={'X-E6-Control-Token':os.environ['E6_TELEOP_HARNESS_TOKEN']},proxy=None) as ws:
            h.ws=ws;hello=json.loads(await ws.recv())
            if not hello.get('ok'):raise RuntimeError('Controller unavailable')
            h.status=hello['status'];h.receiver=asyncio.create_task(h.replies())
            if h.status.get('recovery',{}).get('policy')!='bounded':raise ValueError('Bounded policy is required')
            n=h.neutral();h.fresh_input_session=True
            await h.phase(.3,n);await h.prepare(n)
            row=next(r for r in fixtures['rows'] if r['source']==n['source'] and r['mode']==n['mode'] and r['frame']==n['frame'] and not r['fine'] and r['axis'] in (0,'x') and r['sign']==1)
            for index in range(4):
                await h.phase(.2,row['input'],True)
                # No socket teardown or old input retransmission. Suspend only
                # the sender, so the X5 must detect its unchanged 250ms lease.
                await asyncio.sleep(.4)
                until=time.monotonic()+2
                while h.status['armed']:
                    if time.monotonic()>until:raise RuntimeError('Input watchdog did not stop')
                    await asyncio.sleep(.02)
                if h.status.get('first_fault',{}).get('reason')!='输入看门狗超时':raise RuntimeError('Unexpected protective stop cause')
                await h.phase(5.7,n)
                recovery=h.status['recovery']
                if index<3:
                    if not h.status['armed'] or recovery['state']!='ready':raise RuntimeError('Fresh neutral stream did not automatically prepare')
                    if recovery['total_attempts']!=index+1:raise RuntimeError('Recovery attempt count mismatch')
                    cases.append(dict(index=index+1,automatically_prepared=True,old_target_replayed=False))
                else:
                    if h.status['armed'] or recovery['state']!='blocked' or recovery['total_attempts']!=3:raise RuntimeError('Three-attempt limiter did not lock')
                    cases.append(dict(index=4,automatically_prepared=False,limit_enforced=True))
            await h.rpc(dict(operation='stop'));await h.phase(.5,n)
            if h.status['armed']:raise RuntimeError('Explicit Stop was undone')
            if h.actions or h.invalid:raise RuntimeError('Actions or invalid observations during shadow fault test')
            stats=h.latest.get('control',{}).get('stats',{})
            if any(stats.get(k)!=0 for k in ('motion_tx','stop_tx','can_tx')):raise RuntimeError('Shadow emitted device writes')
            if source_hashes()!=h.source_manifest:raise RuntimeError('Source changed during acceptance')
            passed=True
    except Exception as exc:reason=str(exc)
    finally:
        if h.ws and h.ws.state.name!='CLOSED':
            try:await h.rpc(dict(operation='stop'))
            except Exception:pass
        await asyncio.sleep(.2);observer.cancel();await asyncio.gather(observer,return_exceptions=True)
        if h.pending_evidence:await h.pending_evidence
        h.stream.close()
        if h.receiver:h.receiver.cancel();await asyncio.gather(h.receiver,return_exceptions=True)
        if h.ws:await h.ws.close()
        status={k:v for k,v in (h.status or {}).items() if k not in ('lease','session')}
        summary=dict(passed=passed,reason=reason,stage='shadow',suite='bounded-recovery',duration_s=(time.perf_counter_ns()-started)/1e9,
            frames=h.frames,invalid=h.invalid,actions=h.actions,candidate_ticks=h.candidate_ticks,cases=cases,
            source_sha256=h.source_manifest,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            production_input_sha256=fixtures['production_input_sha256'],stats=(h.latest or {}).get('control',{}).get('stats'),final_status=status,
            physical_motion_test=False,eligible_for_operator=False,old_action_replayed=False)
        (destination/'summary.json').write_bytes((json.dumps(summary,ensure_ascii=False,indent=2)+'\n').encode())
        print(json.dumps({k:summary[k] for k in ('passed','reason','frames','cases')},ensure_ascii=False))
    if not passed:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    fixtures=json.loads(a.fixtures.read_bytes())
    actual=hashlib.sha256((Path(__file__).resolve().parents[1]/'web/src/input.ts').read_bytes()).hexdigest()
    if fixtures['production_input_sha256']!=actual:raise ValueError('Fixtures differ from production mapper')
    if a.evidence.exists():raise ValueError('New evidence directory is required')
    asyncio.run(run(fixtures,a.evidence))
