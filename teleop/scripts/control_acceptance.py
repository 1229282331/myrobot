"""Synthetic production input -> PC lease bridge -> X5. Never bypasses mapping.

Default shadow; live requires an explicitly started live harness session.
All failure frames are retained, and every exit requests Stop.
"""
import argparse,asyncio,copy,hashlib,json,os,time,urllib.request
from pathlib import Path
import numpy as np
import websockets
from e6_teleop.pc_tcp_latency import configuration

ROOT=Path(__file__).resolve().parents[1]

def source_hashes():
    from e6_teleop.acceptance_profile import CRITICAL
    return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in CRITICAL}

def matrix_rows(fixtures):
    # Keep each real mapper-produced multi-axis signal unchanged; the axis
    # label is coverage metadata, not a constructed Cartesian command.
    return fixtures['rows']+[dict(row,axis='xyz') for row in fixtures.get('multi',[])]

def request(path,data=None):
    body=json.dumps(data).encode() if data is not None else None
    req=urllib.request.Request('http://127.0.0.1:8765'+path,data=body,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as response:return json.load(response)

class Harness:
    def __init__(self,fixtures,evidence,diagnostic_transport=False):
        self.fixtures=fixtures;self.evidence=evidence;self.latest=None;self.status=None;self.seq=0;self.ws=None
        self.frames=0;self.candidates=0;self.actions=0;self.latencies=[];self.first=None;self.last_tick=None
        self.tick_gaps=0;self.invalid=0;self.logs=[]
        self.skipped_input_periods=0;self.completed=False;self.failure=None;self.start_ns=None
        self.rpc_timings=[]
        self.protocol_trace=[]
        self.requests={};self.request_id=0;self.receiver=None;self.remote_error=None
        self.last_input_identity=None
        self.fresh_input_session=False
        self.session_transitions=[]
        self.harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.input_periods=[];self.last_sent_ns=None;self.source_manifest=source_hashes()
        self.pending_evidence=None
        self.diagnostic_transport=diagnostic_transport
        self.terminal_error=None;self.seen_ticks=set();self.candidate_ticks=0;self.action_ticks=0
        evidence.mkdir(parents=True,exist_ok=True)
        self.stream=(evidence/'frames.jsonl').open('w',encoding='utf-8')
    async def observe(self):
        async with websockets.connect('ws://127.0.0.1:8765/ws/control-audit',additional_headers={'X-E6-Control-Token':os.environ['E6_TELEOP_HARNESS_TOKEN']},max_size=8*1024*1024,proxy=None) as ws:
            async for message in ws:
                f=json.loads(message);self.latest=f
                if 'observation' not in f:continue
                self.frames+=1;self.invalid+=int(not f['observation']['valid'])
                tick=(f.get('control') or {}).get('tick') or {}
                self.candidates+=int(bool(tick.get('candidate_action')));self.actions+=int(bool(tick.get('effective_action')))
                identity=tick.get('control_tick_id')
                if identity is not None and identity not in self.seen_ticks:
                    self.seen_ticks.add(identity)
                    self.candidate_ticks+=int(bool(tick.get('candidate_action')))
                    self.action_ticks+=int(bool(tick.get('effective_action')))
                s=f.get('status') or {}
                if self.status and s.get('session')==self.status.get('session') and s.get('status_revision_ns',0)>self.status.get('status_revision_ns',0):self.status=s
                if self.first is None and f['observation']['valid']:self.first=copy.deepcopy(f)
                # Diagnostic JSON/disk work must not delay 20 Hz inputs. At
                # most one outstanding write; failures still fail acceptance.
                self.pending_evidence=asyncio.get_running_loop().run_in_executor(None,self.append_evidence,f)
                await asyncio.shield(self.pending_evidence)
    def append_evidence(self,f):
        row=copy.deepcopy({k:v for k,v in f.items() if k not in ('images','preview')})
        for s in (row.get('status'),row.get('control_status'),(row.get('control') or {}).get('status'),((row.get('control') or {}).get('tick') or {}).get('status')):
            if s:
                for k in ('lease','session'):s.pop(k,None)
        for source in row.get('sources',{}).values():
            for k in ('jpeg','uint16_raw','preview_jpeg'):source.pop(k,None)
        self.stream.write(json.dumps(row,ensure_ascii=False)+'\n');self.stream.flush()
    async def send(self,data,future=None):
        if len(self.requests)>=32:raise RuntimeError('Unacknowledged inputs exceed bounded diagnostic window')
        payload=dict(data)
        if payload['operation']=='input':
            self.last_input_identity=(payload['source'],payload['identity'])
            self.seq+=1;payload.update(seq=self.seq,lease=self.status['lease'])
        self.request_id+=1;payload['request_id']=self.request_id
        start=time.perf_counter_ns();self.requests[self.request_id]=(start,payload['operation'],future)
        self.protocol_trace.append(dict(kind='send',request_id=self.request_id,operation=payload['operation'],
            seq=payload.get('seq'),source=payload.get('source'),pc_monotonic_ns=start,
            lease_digest=hashlib.sha256(str(payload.get('lease','')).encode()).hexdigest()))
        if payload['operation']=='input':
            # The archived mapper fixture carries a fixed browser timestamp.
            # Retain it as fixture provenance and record this actual PC sample
            # time separately; never treat timestamp=0 as live acquisition.
            payload['raw']=dict(payload.get('raw',{}),pc_harness_sampled_monotonic_ns=start,
                                fixture_timestamp_semantics='fixed synthetic reference, not acquisition time')
            if self.last_sent_ns is not None:self.input_periods.append((start-self.last_sent_ns)/1e6)
            self.last_sent_ns=start
        await asyncio.wait_for(self.ws.send(json.dumps(payload,allow_nan=False)),.05)
    async def replies(self):
        try:
            async for raw in self.ws:
                r=json.loads(raw);start,operation,future=self.requests.pop(r['request_id'])
                status=r.get('status') or {};kin=status.get('kinematics') or {}
                self.protocol_trace.append(dict(kind='reply',request_id=r['request_id'],operation=operation,
                    ok=r.get('ok'),error=r.get('error'),seq=status.get('seq'),reason=status.get('reason'),
                    status_revision_ns=status.get('status_revision_ns'),connection_ready=kin.get('connection_ready'),
                    connection_age_ms=kin.get('connection_age_ms'),connection_staging=kin.get('connection_staging'),
                    connection_staging_error=kin.get('connection_staging_error'),pc_monotonic_ns=time.perf_counter_ns()))
                self.protocol_trace[-1]['lease_digest']=hashlib.sha256(str(status.get('lease','')).encode()).hexdigest()
                self.latencies.append((time.perf_counter_ns()-start)/1e6)
                if r.get('rpc_timing'):
                    timing=dict(r['rpc_timing'],operation=operation,round_trip_ms=self.latencies[-1])
                    timing['pc_bridge_ms']=(timing['pc_gateway_reply_ns']-timing['pc_gateway_sent_ns'])/1e6
                    timing['transport_and_scheduling_ms']=timing['pc_bridge_ms']-timing['lock_wait_ms']-timing['processing_ms']
                    self.rpc_timings.append(timing)
                if 'status' in r and r['status'].get('status_revision_ns',0)>=self.status.get('status_revision_ns',0):self.status=r['status']
                if not r.get('ok'):
                    error=RuntimeError(r.get('error','Control rejected'))
                    if future and not future.done():future.set_exception(error)
                    else:self.remote_error=error
                elif future and not future.done():future.set_result(r)
        except asyncio.CancelledError:raise
        except Exception as exc:
            self.remote_error=exc
            for _,_,future in self.requests.values():
                if future and not future.done():future.set_exception(exc)
    async def rpc(self,data):
        future=asyncio.get_running_loop().create_future()
        await self.send(data,future)
        return await asyncio.wait_for(future,1)
    async def neutral_rpc(self,data,neutral):
        if neutral['deadman'] or not neutral['neutral']:raise ValueError('RPC heartbeat requires neutral released input')
        task=asyncio.create_task(self.rpc(data))
        try:
            while not task.done():
                done,_=await asyncio.wait({task},timeout=.05)
                if task in done:break
                if self.remote_error:raise self.remote_error
                # Newly sampled neutral inputs, with new sequence/timestamp.
                # Never resend the RPC or an active/old-device input.
                await self.send(neutral)
            return await task
        finally:
            if not task.done():task.cancel();await asyncio.gather(task,return_exceptions=True)
    def neutral(self,source='gamepad',mode='translation',frame='base'):
        return dict(self.fixtures['neutral'][source],mode=mode,frame=frame)
    async def phase(self,seconds,input,require_armed=False):
        identity=(input['source'],input['identity'])
        if identity!=self.last_input_identity:
            if input['deadman'] or not input['neutral']:raise ValueError('Input-device transition requires neutral handshake')
            # A source change revokes its lease. Wait for that one neutral
            # acknowledgement before streaming subsequent inputs, so wireless
            # return delay cannot turn the transition into old-token retries.
            if getattr(self,'fresh_input_session',False):await self.neutral_rpc(input,input)
            else:await self.rpc(input)
            self.fresh_input_session=False
        start=time.monotonic();deadline=start
        while time.monotonic()-start<seconds:
            if self.remote_error:raise self.remote_error
            await self.send(input)
            if require_armed and not self.status['armed']:raise RuntimeError('Control stopped: '+self.status['reason'])
            if self.latest and self.latest.get('stats',{}).get('fatal'):raise RuntimeError(self.latest['stats']['fatal'])
            if self.latest and not self.latest.get('observation',{}).get('valid'):raise RuntimeError('Required observations invalid')
            deadline+=.05
            if time.monotonic()>deadline:
                self.skipped_input_periods+=int((time.monotonic()-deadline)/.05)+1
                deadline=time.monotonic()+.05
            await asyncio.sleep(max(0,deadline-time.monotonic()))
    async def prepare(self,neutral):
        until=time.monotonic()+5
        while (self.status.get('kinematics') or {}).get('connection_ready') is False:
            if time.monotonic()>until:raise RuntimeError('Neutral connection resource staging not ready')
            await self.phase(.1,neutral)
        return await self.neutral_rpc(dict(operation='arm'),neutral)
    async def replace_input_session(self):
        """End the old source's authority before opening a different source.

        Neutral-only shadow transition; no two controllers coexist, and no
        input carrying a revoked old-source lease is ever resubmitted.
        """
        if self.latest.get('control_stage')!='shadow':raise ValueError('Matrix source-session replacement is shadow-only')
        await self.rpc(dict(operation='stop'))
        await self.ws.close()
        self.receiver.cancel();await asyncio.gather(self.receiver,return_exceptions=True)
        self.requests.clear();self.remote_error=None;self.last_input_identity=None
        self.last_sent_ns=None
        # Closing the local WebSocket does not mean the PC's upstream bridge
        # has released its X5 owner yet. Only retry a rejected, neutral-only
        # connection; never retry an input, arm, or device transaction.
        deadline=time.monotonic()+2;attempts=0
        while True:
            await asyncio.sleep(.2)
            attempts+=1
            self.ws=await websockets.connect('ws://127.0.0.1:8765/ws/control',
                additional_headers={'X-E6-Control-Token':os.environ['E6_TELEOP_HARNESS_TOKEN']},proxy=None)
            hello=json.loads(await asyncio.wait_for(self.ws.recv(),max(.001,deadline-time.monotonic())))
            if hello.get('ok'):break
            await self.ws.close()
            if hello.get('error')!='已有控制者' or time.monotonic()>=deadline:
                raise RuntimeError(str(hello))
        self.session_transitions.append(dict(attempts=attempts,pc_completed_monotonic_ns=time.perf_counter_ns()))
        self.status=hello['status'];self.receiver=asyncio.create_task(self.replies())
        self.fresh_input_session=True
    async def run(self,stage,seconds,suite):
        self.start_ns=time.perf_counter_ns()
        token=os.environ['E6_TELEOP_HARNESS_TOKEN'];observer=asyncio.create_task(self.observe())
        def observation_failed(task):
            if not task.cancelled() and task.exception():self.remote_error=task.exception()
        observer.add_done_callback(observation_failed)
        try:
            deadline=time.monotonic()+15
            while self.first is None:
                if self.remote_error:raise self.remote_error
                if (self.latest or {}).get('stats',{}).get('fatal'):raise RuntimeError(self.latest['stats']['fatal'])
                if time.monotonic()>deadline:raise RuntimeError('Observations not ready: '+str((self.latest or {}).get('pc_error')))
                await asyncio.sleep(.1)
            if self.latest.get('control_stage')!=stage:raise RuntimeError('Stage mismatch')
            async with websockets.connect('ws://127.0.0.1:8765/ws/control',additional_headers={'X-E6-Control-Token':token},proxy=None) as ws:
                self.ws=ws;hello=json.loads(await ws.recv())
                if not hello.get('ok'):raise RuntimeError(str(hello))
                self.status=hello['status'];n=self.neutral();self.fresh_input_session=True
                self.receiver=asyncio.create_task(self.replies())
                await self.rpc(dict(operation='stop'))
                start=time.monotonic()
                if suite=='diagnose':
                    await self.rpc(dict(operation='diagnose-kinematics'))
                    while True:
                        await self.phase(.2,n)
                        task=self.status.get('commissioning') or {}
                        if task.get('state')=='failed':raise RuntimeError(task['error'])
                        if task.get('state')=='complete':break
                    self.completed=True;return
                if stage=='live' and suite=='commission':
                    await self.rpc(dict(operation='enable'))
                    await self.phase(6,n)
                    if (self.latest.get('status',{}).get('commissioning') or {}).get('state')!='complete':raise RuntimeError('Enable not completed')
                    # Enable is an independent step. The newly enabled anchor
                    # must be sampled/validated before any zero-increment ServoJ.
                    self.completed=True;return
                await self.phase(.2,n);await self.prepare(n)
                # First live motion always uses zero increments through the mapper.
                # Unknown gripper remains unknown and no CAN is sent at this step.
                if stage=='live' or suite=='zero':
                    await self.phase(.1,dict(n,deadman=True),True)
                    await self.phase(1,n,True)
                    if self.actions==0 and stage=='live':raise RuntimeError('No acknowledged zero-increment ServoJ observed')
                if suite not in ('neutral','zero','commission'):
                    rows=matrix_rows(self.fixtures) if suite=='matrix' else self.fixtures['rows'][:1]
                    # A matrix stress run continues calculating candidates,
                    # instead of spending its remaining ten minutes neutral.
                    i=0
                    while i<len(rows) or suite=='matrix' and time.monotonic()-start<seconds:
                        row=rows[i%len(rows)]
                        if self.last_input_identity and self.last_input_identity!=(row['input']['source'],row['input']['identity']):
                            await self.replace_input_session()
                        n=self.neutral(row['source'],row['mode'],row['frame'])
                        await self.phase(.15,n)
                        if not self.status['armed']:await self.prepare(n)
                        await self.phase(.35,row['input'],True)
                        await self.phase(.15,n,True)
                        self.logs.append({k:row[k] for k in ('source','mode','frame','fine','axis','sign')})
                        i+=1
                        if i%16==0:print(json.dumps(dict(cases=i,elapsed_s=round(time.monotonic()-start,1))),flush=True)
                n=self.neutral(n['source'],n['mode'],n['frame'])
                while time.monotonic()-start<seconds:
                    await self.phase(min(5,seconds-(time.monotonic()-start)),n,True)
                    if int(time.monotonic()-start)%60<5:print(json.dumps(dict(elapsed_s=round(time.monotonic()-start,1),frames=self.frames,candidates=self.candidates)),flush=True)
                await self.rpc(dict(operation='stop'))
                await asyncio.sleep(.3)
                if self.invalid:raise RuntimeError('Required source invalid during shadow acceptance')
                if source_hashes()!=self.source_manifest:raise RuntimeError('Acceptance source changed during run')
                self.completed=True
        except Exception as exc:
            self.terminal_error=str(exc)
            first=(self.status or {}).get('first_fault')
            self.failure=first['reason'] if first else self.terminal_error
            raise
        finally:
            if self.ws and not self.ws.state.name=='CLOSED' and (self.status or {}).get('stop_acknowledged') is not False:
                try:await self.rpc(dict(operation='stop'))
                except Exception:pass
            await asyncio.sleep(.3)
            observer.cancel();await asyncio.gather(observer,return_exceptions=True)
            if self.pending_evidence:
                try:await self.pending_evidence
                except Exception as exc:self.completed=False;self.failure=self.failure or str(exc)
            self.stream.close()
            if self.receiver:
                self.receiver.cancel();await asyncio.gather(self.receiver,return_exceptions=True)
            if self.ws:await self.ws.close()
            summary=dict(passed=self.completed,failure=self.failure,duration_s=(time.perf_counter_ns()-self.start_ns)/1e9,suite=suite,production_input_sha256=self.fixtures['production_input_sha256'],stage=stage,frames=self.frames,invalid=self.invalid,candidates=self.candidates,actions=self.actions,cases=self.logs,skipped_input_periods=self.skipped_input_periods,
                         wifi_latency=(self.latest or {}).get('stats',{}).get('wifi_latency'),
                         ssh_tcp_latency=(self.latest or {}).get('stats',{}).get('ssh_tcp_latency'),
                         e6_tcp_latency=(self.latest or {}).get('stats',{}).get('e6_tcp_latency'),
                         terminal_error=self.terminal_error,candidate_ticks=self.candidate_ticks,action_ticks=self.action_ticks,
                         diagnostic_transport=self.diagnostic_transport,
                         counter_semantics='candidates/actions are observation publication counts; *_ticks count unique guardian ticks',
                         test_fixture=False,hardware_sources_verified=bool(self.first and self.first['sources']['robot'].get('source') in ('E6:30004','E6:30004 guardian') and self.first['sources']['robot'].get('raw_base64')),
                         source_sha256=self.source_manifest,source_changed_during_run=source_hashes()!=self.source_manifest,
                         e6_network=(self.latest or {}).get('stats',{}).get('e6_network'),
                         pc_tcp_configuration=configuration((self.latest or {}).get('stats',{}).get('pc_tcp_latency') or {}),
                         media_hz=(self.latest or {}).get('stats',{}).get('sampling',{}).get('preview_hz'),
                         harness_sha256=self.harness_sha256,
                         session_transitions=self.session_transitions,
                         input_period_ms={k:float(np.percentile(self.input_periods,v)) for k,v in [('p50',50),('p95',95),('p99',99),('max',100)]} if self.input_periods else {},
                         latency_ms={k:float(np.percentile(self.latencies,v)) for k,v in [('p50',50),('p95',95),('p99',99),('max',100)]} if self.latencies else {},
                         stats=(self.latest or {}).get('control',{}).get('stats'),final_status=self.status)
            if summary['final_status']:
                summary['final_status']={k:v for k,v in summary['final_status'].items() if k not in ('session','lease')}
            (self.evidence/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
            (self.evidence/'rpc_timings.json').write_text(json.dumps(self.rpc_timings,indent=2),encoding='utf-8')
            (self.evidence/'protocol_trace.json').write_text(json.dumps(self.protocol_trace,indent=2),encoding='utf-8')

def shadow_sources_match(result,compatibility=None):
    """Explicit regression reuse for lossless framing only, never motion code."""
    current=source_hashes();original=result.get('source_sha256') or {}
    if original==current:return True
    compatibility=compatibility or os.environ.get('E6_TELEOP_SHADOW_COMPATIBILITY')
    if not compatibility:return False
    p=Path(compatibility).resolve();review=json.loads(p.read_bytes())
    if (review.get('passed') is not True or review.get('scope')!='lossless_observation_header_only'
            or review.get('original_source_sha256')!=original or review.get('current_source_sha256')!=current
            or review.get('control_limits_or_deadlines_changed') is not False
            or set(original)!=set(current)
            or [k for k in current if current[k]!=original[k]]!=['device_gateway/e6_observer/protocol.py']):return False
    # The complete original shadow, source diff and offline roundtrip/tests
    # remain separate files with original provenance, never relabelled runs.
    if not review.get('proofs'):return False
    for proof in review['proofs']:
        file=(p.parent/proof['file']).resolve()
        if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest()!=proof['sha256']:return False
    return True


def require_shadow_acceptance(path,fixtures,compatibility=None):
    if path is None:raise ValueError('Live harness requires a passing 600-second shadow matrix; no enable or CAN sent')
    result=json.loads(Path(path).read_text(encoding='utf-8'))
    stats=result.get('stats') or {}
    coverage=lambda rows:{tuple(row[k] for k in ('source','mode','frame','fine','axis','sign')) for row in rows}
    if (result.get('passed') is not True or result.get('stage')!='shadow' or result.get('suite')!='matrix'
        or result.get('test_fixture') is not False or result.get('hardware_sources_verified') is not True
        or result.get('diagnostic_transport',False)
        or result.get('experiment_only',False) or result.get('eligible_for_hardware_acceptance') is False
        or result.get('duration_s',0)<600 or result.get('invalid',1)!=0 or result.get('actions',1)!=0
        or result.get('production_input_sha256')!=fixtures['production_input_sha256']
        or not shadow_sources_match(result,compatibility)
        or result.get('source_changed_during_run') is not False
        or coverage(result.get('cases',[]))!=coverage(matrix_rows(fixtures))
        or any(((result.get('final_status') or {}).get('limits') or {}).get(k)!=v for k,v in fixtures.get('profile',{}).items())
        or (fixtures.get('gripper_command_profile') is not None and ((result.get('final_status') or {}).get('gripper_command_profile') or {}).get('name')!=fixtures['gripper_command_profile'])
        or stats.get('motion_tx')!=0 or stats.get('can_tx')!=0 or stats.get('stop_tx')!=0):
        raise ValueError('Shadow acceptance incomplete/failed; live harness remains locked')

def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',choices=('shadow','live'),default='shadow');p.add_argument('--seconds',type=float,default=600)
    p.add_argument('--suite',choices=('matrix','commission','zero','single','diagnose','neutral'),default='matrix');p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--shadow-evidence',type=Path)
    p.add_argument('--diagnostic-transport',action='store_true',help='Shadow only; result cannot unlock live acceptance')
    a=p.parse_args()
    if a.stage=='live' and a.diagnostic_transport:p.error('Diagnostic transport is shadow-only')
    fixtures=json.loads(a.fixtures.read_text());sha=hashlib.sha256((ROOT/'web/src/input.ts').read_bytes()).hexdigest()
    if fixtures['production_input_sha256']!=sha:raise ValueError('Fixtures do not match production input source')
    if a.stage=='live':require_shadow_acceptance(a.shadow_evidence,fixtures)
    h=Harness(fixtures,a.evidence,a.diagnostic_transport)
    try:asyncio.run(h.run(a.stage,a.seconds,a.suite))
    except Exception as exc:
        (a.evidence/'failure.txt').write_text(str(exc),encoding='utf-8');raise

if __name__=='__main__':main()
