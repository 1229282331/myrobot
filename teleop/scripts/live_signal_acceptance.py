"""Gated empty-load tests through production input mapping and both bridges.

No direct joint target, enable, replay, parameter write or automatic recovery.
Each invocation retains its complete audit, even when a protection stops it.
"""
import argparse
import asyncio
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np
import websockets
from control_acceptance import Harness,request,require_shadow_acceptance,source_hashes
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.control_math import angle,matrix,pose,multiply
from e6_observer.local_kinematics import logarithm,inverse


def final_stop_status(reply_status, observed_status):
    """Keep the newest acknowledgement of this exact stopped request.

    Closing the owned input socket clears its owner, while the independent
    observation stream can still receive the pending Stop acknowledgement.
    A different controller or Stop request must not replace this result.
    """
    reply_status=reply_status or {}
    observed_status=observed_status or {}
    if (observed_status.get('session') in (None,reply_status.get('session'))
            and observed_status.get('stop_requested_ns')==reply_status.get('stop_requested_ns')
            and observed_status.get('status_revision_ns',0)>=reply_status.get('status_revision_ns',0)):
        return observed_status
    return reply_status


def axis_cases(suite,frame_filter=None,fine_filter=None,mode_filter=None,axis_filter=None):
    if suite not in ('axes','frames-fine'):
        if any(v is not None for v in (frame_filter,fine_filter,mode_filter,axis_filter)):
            raise ValueError('Axis filters require axes or frames-fine suite')
        return []
    frames=('base',) if suite=='axes' else ('base','tool')
    fines=(False,) if suite=='axes' else (False,True)
    if frame_filter is not None:
        if frame_filter not in frames:raise ValueError('Frame is outside the selected suite')
        frames=(frame_filter,)
    if fine_filter is not None:
        fine={'normal':False,'fine':True}[fine_filter]
        if fine not in fines:raise ValueError('Precision is outside the selected suite')
        fines=(fine,)
    modes=('translation','rotation')
    axes=tuple(range(3))
    if mode_filter is not None:
        if mode_filter not in modes:raise ValueError('Unknown Cartesian mode')
        modes=(mode_filter,)
    if axis_filter is not None:
        if axis_filter not in axes:raise ValueError('Unknown Cartesian axis')
        axes=(axis_filter,)
    return [(frame,fine,mode,axis) for frame in frames for fine in fines
            for mode in modes for axis in axes]


def transmitted_gripper_goal(tick,nominal,profile,*,previous_action=None,holding_feedback=None):
    """Check arrival against the actual byte goal, retaining quantization."""
    action=tick.get('last_accepted_action') or {}
    provenance='accepted_control_transaction'
    if action.get('gripper_acceptance')=='suppressed_due_to_deadman_release':
        # The released transaction never sent CAN. Audit the preceding
        # accepted byte, only when a new hold reply proves that same goal.
        feedback=holding_feedback or {}
        if (not feedback.get('valid') or feedback.get('fault') or feedback.get('state') in (2,3)
                or feedback.get('source')!='can1 request-associated feedback, not cryptographic authentication'
                or 'requested_opening' not in feedback or 'command_opening' not in feedback):
            raise ValueError('Released gripper target has no fresh matching hold feedback')
        # The idle CAN hold has its own request and exact encoded goal. It
        # need not share a ServoJ transaction whose CAN write was suppressed.
        # This validates the endpoint; it never fills that action's CAN label.
        action=dict(gripper_acceptance='request_associated_feedback',gripper=feedback['command_opening'],
            requested_gripper=feedback['requested_opening'],gripper_feedback=feedback)
        provenance='neutral_hold_request_associated_can'
    if action.get('gripper_acceptance')!='request_associated_feedback':
        raise ValueError('Final gripper target was not accepted with associated feedback')
    feedback=action.get('gripper_feedback') or {}
    raw=feedback.get('command_position_raw');effective=action.get('gripper')
    if type(raw) is not int or not 0<=raw<=255 or effective!=raw/255:
        raise ValueError('Final gripper target differs from transmitted byte')
    requested=action.get('requested_gripper')
    resolution=profile.get('minimum_target_delta_counts',1)
    if requested is None or abs(effective-requested)>(resolution-.5)/255+1e-9:
        raise ValueError('Final gripper quantization exceeds the configured resolution')
    # Integration is clamped to at most 75 ms even when a 50 ms tick jitters.
    if abs(requested-nominal)>profile['gripper_s']*.075+1e-9:
        raise ValueError('Final requested gripper endpoint was not reached')
    return dict(effective_gripper_target=effective,requested_gripper_target=requested,
        nominal_gripper_target=nominal,command_position_raw=raw,
        requested_to_effective_counts=(requested-effective)*255,endpoint_provenance=provenance)


class LiveSignals(Harness):
    def __init__(self,fixtures,evidence,shadow):
        if evidence.exists():raise ValueError('Use a new evidence directory; previous runs are immutable')
        require_shadow_acceptance(shadow,fixtures)
        self.shadow=json.loads(shadow.read_text(encoding='utf-8'))
        self.checks=[];self.episode=None;self.recording=False
        self.last_gripper_action=None
        super().__init__(fixtures,evidence)

    def append_evidence(self,frame):
        action=((frame.get('control') or {}).get('tick') or {}).get('last_accepted_action') or {}
        if action.get('gripper_acceptance')=='request_associated_feedback':
            self.last_gripper_action=copy.deepcopy(action)
        super().append_evidence(frame)

    def row(self,source,mode,frame,axis,sign,fine=False):
        # The fixtures were executed by actual fromGamepad/fromKeyboard, not
        # constructed Cartesian commands or direct joint positions here.
        return copy.deepcopy(next(v['input'] for v in self.fixtures['rows'] if
            (v['source'],v['mode'],v['frame'],v['axis'],v['sign'],v['fine'])==(source,mode,frame,axis,sign,fine)))

    def actual(self):return (self.latest or {})['sources']['robot']['decoded']

    async def start_recording(self,source,neutral):
        # HTTP archive setup is independent of control. Keep freshly sampled
        # released input flowing while it completes, just as the browser's
        # input timer does; never send an active increment or retry the HTTP.
        task=asyncio.create_task(asyncio.to_thread(request,'/api/record/start',
            dict(task='empty-load '+source+' production mapped synthetic input')))
        try:
            while not task.done():
                done,_=await asyncio.wait({task},timeout=.05)
                if task in done:break
                if self.remote_error:raise self.remote_error
                await self.send(neutral)
            response=await task
            self.episode=response['episode'];self.recording=True
            return response
        except BaseException:
            # Cancelling to_thread does not cancel the HTTP archive start.
            # Stop input first, then retain a late successful response so
            # run_live's exit path can finish this exact episode as aborted.
            try:await self.rpc(dict(operation='stop'))
            except Exception:pass
            try:
                response=await asyncio.shield(task)
                self.episode=response['episode'];self.recording=True
            except Exception:pass
            raise

    def gripper_signal(self,source,sign,target):
        candidate=(self.latest.get('control',{}).get('tick') or {}).get('candidate_action') or {}
        goal=candidate.get('gripper')
        # Keep the medium-rate travel. Use the real mapper's precision input
        # near a measured target, so observation-return latency cannot turn a
        # nominal 35% endpoint into an untested 36% endpoint. No target is sent
        # directly, and arrival/quantization tolerances remain unchanged.
        fine=getattr(self,'gripper_fine',False) or goal is not None and abs(target-goal)<.03
        return self.row(source,'translation','base',3,sign,fine)

    def candidate_pose(self):
        tick=(self.latest or {}).get('control',{}).get('tick') or {}
        candidate=tick.get('candidate_action')
        raw=(candidate or {}).get('requested_tcp',{}).get('controller_pose_mm_deg')
        return pose(raw or self.actual()['controller_pose_raw_mm_deg'])

    async def prepare(self,source,mode='translation',frame='base'):
        neutral=self.neutral(source,mode,frame)
        await self.phase(.25,neutral)
        if not self.status['armed']:await super().prepare(neutral)
        if not self.status['armed']:raise RuntimeError('Explicit preparation failed')
        return neutral

    async def settled(self,neutral):
        # Release once, then send fresh neutral samples only. A fixed sleep
        # can land on a transient nonstationary sample even after Stop ACK.
        # This bounded observation wait changes no device/watchdog deadline
        # and never sends another Stop, arm or active input.
        started=time.monotonic()
        previous_stop=self.status.get('stop_requested_ns') or 0
        release_seq=self.seq+1
        for _ in range(60):
            await self.phase(.05,neutral,True)
            s=self.latest.get('status',self.status)
            stopped=s.get('stop_requested_ns') or 0
            received=self.latest.get('sources',{}).get('robot',{}).get('x5_receive_monotonic_ns',0)
            if (s.get('seq',-1)>=release_seq and stopped>previous_stop and received>stopped
                    and s.get('stop_acknowledged') is True and s.get('stopped_feedback')):
                break
        else:
            raise RuntimeError('Stop acknowledgement and stationary feedback not both confirmed')
        q=np.asarray(self.actual()['joints_rad'])
        return dict(joints_rad=q.tolist(),pose_mm_deg=self.actual()['controller_pose_raw_mm_deg'],
                    stop_acknowledged=True,stopped_feedback=True,stop_requested_ns=stopped,
                    neutral_observation_wait_s=time.monotonic()-started)

    async def increment(self,source,mode,frame,axis,sign,amount,origin,fine=False,returning=False):
        neutral=await self.prepare(source,mode,frame);signal=self.row(source,mode,frame,axis,sign,fine)
        p0,r0=pose(origin);direction=(np.eye(3) if frame=='base' else matrix(r0))[:,axis]
        threshold=.00006 if mode=='translation' else math.radians(.02)
        start=time.monotonic();deadline=start;baseline_tick=(self.latest.get('control',{}).get('tick') or {}).get('control_tick_id')
        final_progress=None
        while True:
            # At 30% stick + 25% fine, a 1 mm move needs >12 s when a
            # pending ServoJ skips alternate periods. No cycle is caught up.
            if time.monotonic()-start>30:raise TimeoutError('Input-driven bounded increment timed out')
            await self.phase(.05,signal,True)
            tick=(self.latest.get('control',{}).get('tick') or {}).get('control_tick_id')
            if tick is None or tick==baseline_tick:continue
            pp,rr=self.candidate_pose()
            delta=pp-p0 if mode=='translation' else logarithm(multiply(rr,inverse(r0)))
            progress=float(delta@direction)
            final_progress=progress
            if returning and sign*progress>=-threshold or not returning and sign*progress>=amount:
                break
        last=await self.settled(neutral)
        return dict(source=source,mode=mode,frame=frame,axis=axis,sign=sign,fine=fine,returning=returning,
                    desired_m_or_rad=amount,requested_projection_m_or_rad=final_progress,
                    actual=last,duration_s=time.monotonic()-start)

    async def round_trip(self,source,mode,frame,axis,amount,fine=False):
        await self.prepare(source,mode,frame)
        origin=copy.deepcopy(self.actual()['controller_pose_raw_mm_deg']);q0=np.asarray(self.actual()['joints_rad'])
        outward=await self.increment(source,mode,frame,axis,1,amount,origin,fine)
        p0,r0=pose(origin);op,orr=pose(outward['actual']['pose_mm_deg'])
        direction=(np.eye(3) if frame=='base' else matrix(r0))[:,axis]
        displacement=op-p0 if mode=='translation' else logarithm(multiply(orr,inverse(r0)))
        projection=float(displacement@direction);perpendicular=float(np.linalg.norm(displacement-projection*direction))
        tolerance=.0005 if mode=='translation' else math.radians(.1)
        if projection<amount*.5 or abs(projection-amount)>tolerance or perpendicular>tolerance:
            raise ValueError('Actual controller feedback did not follow the commanded axis/direction')
        outward['actual_direction_check']=dict(projection_m_or_rad=projection,perpendicular_m_or_rad=perpendicular)
        returned=await self.increment(source,mode,frame,axis,-1,amount,origin,fine,True)
        p0,r0=pose(origin);pp,rr=pose(self.actual()['controller_pose_raw_mm_deg'])
        error=dict(position_m=float(np.linalg.norm(pp-p0)),rotation_rad=angle(rr,r0),
                   joint_max_rad=float(np.max(np.abs(np.asarray(self.actual()['joints_rad'])-q0))))
        if error['position_m']>.001 or error['rotation_rad']>math.radians(.15) or error['joint_max_rad']>math.radians(.15):
            raise ValueError('Input-driven return did not recover new baseline within 1 mm / .15 degrees')
        if mode=='rotation' and frame=='tool' and axis==2:
            delta=np.asarray(outward['actual']['joints_rad'])-q0
            if np.max(np.abs(delta[:5]))>math.radians(.05) or not math.radians(.15)<=abs(delta[5])<=math.radians(.5):
                raise ValueError('Tool Z rotation was not an isolated approximately .25-degree J6 step')
        self.checks.append(dict(passed=True,outward=outward,returned=returned,return_residual=error))
        print(json.dumps(dict(checks=len(self.checks),source=source,mode=mode,frame=frame,axis=axis,fine=fine)),flush=True)

    async def gripper(self,initialized=False,full_range=False):
        source='keyboard' if full_range else 'gamepad'
        n=self.neutral(source);await self.rpc(dict(operation='stop'));await self.phase(.3,n)
        if initialized:
            if not self.status.get('gripper_initialized') or self.status.get('gripper_faulted'):
                raise ValueError('Independent gripper cycle requires a completed nonfaulted initialization')
        else:
            await self.rpc(dict(operation='initialize-gripper'))
            until=time.monotonic()+20
            while True:
                await self.phase(.1,n)
                task=self.status.get('commissioning') or {}
                if task.get('state')=='failed':raise RuntimeError(task['error'])
                if task.get('state')=='complete':break
                if time.monotonic()>until:raise TimeoutError('Independent gripper initialization timeout')
        # Stop clears the arm's stability history. Initialization may finish
        # sooner than five seconds; preserve the production preparation gate.
        await self.phase(5.5,n)
        targets=((.15,-1),) if initialized else ()
        targets+=((.60 if full_range else .35,1),(.15,-1))
        for target,sign in targets:
            await self.prepare(source);signal=self.row(source,'translation','base',3,sign)
            until=time.monotonic()+80
            while True:
                signal=self.gripper_signal(source,sign,target)
                await self.phase(.05,signal,True)
                candidate=(self.latest.get('control',{}).get('tick') or {}).get('candidate_action') or {}
                goal=candidate.get('gripper')
                if goal is not None and sign*(goal-target)>=-1e-9:break
                if time.monotonic()>until:raise TimeoutError('Continuous gripper input timeout')
            await self.settled(n)
            for _ in range(20):
                if (self.latest.get('sources',{}).get('gripper') or {}).get('valid'):break
                await self.phase(.05,n,True)
            endpoints=transmitted_gripper_goal(self.latest['control']['tick'],target,self.status['gripper_command_profile'],
                previous_action=self.last_gripper_action,holding_feedback=self.latest.get('sources',{}).get('gripper'))
            settled=await self.gripper_settled(n,endpoints['effective_gripper_target'])
            self.checks.append(dict(passed=True,gripper_target=target,**endpoints,**settled))

    async def gripper_settled(self,neutral,target):
        # Arm Stop completion is not gripper completion. Observe the existing
        # idle hold at its unchanged final goal for a bounded six seconds.
        # Only neutral input is sent; never retry a failed CAN transaction,
        # synthesize an endpoint or count repeated cached replies as arrivals.
        started=time.monotonic();last_seq=None;stable=0;feedback={}
        for _ in range(120):
            await self.phase(.05,neutral,True)
            feedback=self.latest.get('sources',{}).get('gripper') or {}
            if feedback.get('fault') or feedback.get('state') in (2,3):
                raise ValueError('Gripper fault while observing final hold; no retry')
            if feedback.get('valid') and feedback.get('seq')!=last_seq:
                last_seq=feedback.get('seq')
                stable=stable+1 if feedback.get('state')==0 and abs(feedback.get('opening',-1)-target)<=2/255 else 0
                if stable>=3:return dict(feedback=copy.deepcopy(feedback),gripper_neutral_hold_wait_s=time.monotonic()-started)
        raise ValueError('Gripper target feedback did not settle during six-second unchanged hold; no retry')

    async def multi_round_trip(self,source,mode,frame,fine=False):
        """Three simultaneous axes from the production mapper, never composed twists."""
        await self.prepare(source,mode,frame)
        origin=copy.deepcopy(self.actual()['controller_pose_raw_mm_deg']);q0=np.asarray(self.actual()['joints_rad'])
        p0,r0=pose(origin);basis=np.eye(3) if frame=='base' else matrix(r0)
        direction=basis@np.ones(3)/math.sqrt(3)
        amount=.001 if mode=='translation' else math.radians(.25)
        threshold=.00006 if mode=='translation' else math.radians(.02)
        events=[]
        for sign in (1,-1):
            row=next(v for v in self.fixtures['multi'] if (v['source'],v['mode'],v['frame'],v['fine'],v['sign'])==(source,mode,frame,fine,sign))
            signal=copy.deepcopy(row['input']);start=time.monotonic()
            while True:
                await self.phase(.05,signal,True)
                pp,rr=self.candidate_pose();delta=pp-p0 if mode=='translation' else logarithm(multiply(rr,inverse(r0)))
                progress=float(delta@direction)
                if sign==1 and progress>=amount or sign==-1 and progress<=threshold:break
                if time.monotonic()-start>20:raise TimeoutError('Production multi-axis bounded increment timed out')
            settled=await self.settled(self.neutral(source,mode,frame))
            pp,rr=pose(settled['pose_mm_deg']);delta=pp-p0 if mode=='translation' else logarithm(multiply(rr,inverse(r0)))
            projection=float(delta@direction);perpendicular=float(np.linalg.norm(delta-projection*direction))
            tolerance=.0005 if mode=='translation' else math.radians(.1)
            if sign==1 and (projection<amount*.5 or abs(projection-amount)>tolerance or perpendicular>tolerance):
                raise ValueError('Actual multi-axis direction mismatch')
            events.append(dict(sign=sign,actual=settled,projection_m_or_rad=projection,perpendicular_m_or_rad=perpendicular))
        pp,rr=pose(self.actual()['controller_pose_raw_mm_deg'])
        error=dict(position_m=float(np.linalg.norm(pp-p0)),rotation_rad=angle(rr,r0),joint_max_rad=float(np.max(np.abs(np.asarray(self.actual()['joints_rad'])-q0))))
        if error['position_m']>.001 or error['rotation_rad']>math.radians(.15) or error['joint_max_rad']>math.radians(.15):raise ValueError('Multi-axis return did not recover baseline')
        self.checks.append(dict(passed=True,multi_axes=[0,1,2],source=source,mode=mode,frame=frame,fine=fine,events=events,return_residual=error))
        print(json.dumps(dict(checks=len(self.checks),multi=True,source=source,mode=mode,frame=frame,fine=fine)),flush=True)

    async def execute(self,suite):
        source='keyboard' if suite in ('record-keyboard','gripper-full-range') else 'gamepad'
        n=self.neutral(source)
        await self.phase(5.5,n)
        await self.prepare(source)
        if suite=='zero':
            q0=np.asarray(self.actual()['joints_rad']);before=self.action_ticks
            await self.phase(.1,dict(n,deadman=True),True);await self.settled(n)
            delta=float(np.max(np.abs(np.asarray(self.actual()['joints_rad'])-q0)))
            if self.action_ticks==before or delta>math.radians(.05):raise ValueError('Zero ServoJ acknowledgement or stationary check failed')
            self.checks.append(dict(zero_servoj=True,passed=True,actual_joint_change_max_rad=delta));return
        if suite=='j6':await self.round_trip(source,'rotation','tool',2,math.radians(.25));return
        if suite in ('gripper','gripper-cycle','gripper-full-range'):
            await self.gripper(initialized=suite!='gripper',full_range=suite=='gripper-full-range');return
        if suite=='multi':
            for frame in ('base','tool'):
                for mode in ('translation','rotation'):
                    for fine in (False,True):await self.multi_round_trip(source,mode,frame,fine)
            return
        if suite.startswith('record-'):
            response=await self.start_recording(source,n)
            self.episode=response['episode'];self.recording=True
            begin=time.monotonic()
            while time.monotonic()-begin<30:
                await self.round_trip(source,'translation','base',0,.0003,True)
            await self.rpc(dict(operation='stop'))
            await asyncio.to_thread(request,'/api/record/finish',dict(result='success'));self.recording=False
            return
        for frame,fine,mode,axis in axis_cases(suite,**getattr(self,'axis_selection',{})):
            await self.round_trip(source,mode,frame,axis,.001 if mode=='translation' else math.radians(.25),fine)

    async def run_live(self,suite):
        self.start_ns=time.perf_counter_ns();observer=asyncio.create_task(self.observe());self.ws=None
        def observation_failed(task):
            if not task.cancelled() and task.exception():self.remote_error=task.exception()
        observer.add_done_callback(observation_failed)
        try:
            until=time.monotonic()+15
            while self.first is None:
                if observer.done():await observer
                if time.monotonic()>until:raise TimeoutError('Live observations not ready')
                await asyncio.sleep(.1)
            if self.latest.get('control_stage')!='live' or self.actual()['robot_mode']!=5 or self.actual()['enable_status']!=1:
                raise ValueError('Already enabled, idle live session required; this test never enables')
            expected=(self.shadow.get('final_status') or {}).get('kinematics',{}).get('profile_sha256')
            actual=(self.latest.get('status') or {}).get('kinematics',{}).get('profile_sha256')
            if expected!=actual:raise ValueError('Current calibration differs from passing shadow evidence')
            if self.shadow['final_status']['limits']!=(self.latest.get('status') or {}).get('limits'):raise ValueError('Current control limits differ from passing shadow evidence')
            if self.shadow['final_status'].get('gripper_command_profile')!=(self.latest.get('status') or {}).get('gripper_command_profile'):raise ValueError('Current CAN profile differs from passing shadow evidence')
            if self.shadow.get('e6_network')!=self.latest.get('stats',{}).get('e6_network'):raise ValueError('Current E6 link differs from passing shadow evidence')
            if self.shadow.get('media_hz')!=self.latest.get('stats',{}).get('sampling',{}).get('preview_hz'):raise ValueError('Current preview rate differs from passing shadow evidence')
            from e6_teleop.pc_tcp_latency import configuration
            if self.shadow.get('pc_tcp_configuration')!=configuration(self.latest.get('stats',{}).get('pc_tcp_latency') or {}):raise ValueError('Current PC TCP configuration differs from passing shadow evidence')
            current_wifi=self.latest.get('stats',{}).get('wifi_latency') or {}
            shadow_wifi=self.shadow.get('wifi_latency') or {}
            for field in ('requested','reorder_requested_ms','reorder_effective_ms','driver_ps_on'):
                if current_wifi.get(field)!=shadow_wifi.get(field):raise ValueError('Current Wi-Fi setting differs from passing shadow evidence')
            current_tcp=self.latest.get('stats',{}).get('ssh_tcp_latency') or {}
            shadow_tcp=self.shadow.get('ssh_tcp_latency') or {}
            for field in ('requested_min_rto_ms','applied','peer','source'):
                if current_tcp.get(field)!=shadow_tcp.get(field):raise ValueError('Current SSH TCP setting differs from passing shadow evidence')
            current_e6=self.latest.get('stats',{}).get('e6_tcp_latency') or {}
            shadow_e6=self.shadow.get('e6_tcp_latency') or {}
            for field in ('requested_ms','changed','applied_route'):
                if current_e6.get(field)!=shadow_e6.get(field):raise ValueError('Current E6 TCP setting differs from passing shadow evidence')
            token=os.environ['E6_TELEOP_HARNESS_TOKEN']
            async with websockets.connect('ws://127.0.0.1:8765/ws/control',proxy=None,additional_headers={'X-E6-Control-Token':token}) as ws:
                self.ws=ws;hello=json.loads(await ws.recv())
                if not hello.get('ok'):raise RuntimeError(str(hello))
                self.status=hello['status'];self.receiver=asyncio.create_task(self.replies());self.fresh_input_session=True
                await self.rpc(dict(operation='stop'))
                await self.execute(suite)
                await self.rpc(dict(operation='stop'));await asyncio.sleep(.3)
                if self.remote_error:raise self.remote_error
                if self.invalid:raise ValueError('Live acceptance observed invalid required sources')
                if source_hashes()!=self.source_manifest:raise ValueError('Acceptance source changed during live run')
                self.completed=True
        except BaseException as exc:
            self.failure=((self.status or {}).get('first_fault') or {}).get('reason') or str(exc);raise
        finally:
            if self.ws and self.ws.state.name!='CLOSED' and (self.status or {}).get('stop_acknowledged') is not False:
                try:await self.rpc(dict(operation='stop'))
                except Exception:pass
            if self.recording:
                try:await asyncio.to_thread(request,'/api/record/finish',dict(result='aborted'))
                except Exception:pass
            # Keep read-only audit flowing after a stopped/failed test so that
            # acknowledgement and subsequent stationary feedback are retained.
            # No further input, rearming or motion command is sent here.
            await asyncio.sleep(2)
            observer.cancel();await asyncio.gather(observer,return_exceptions=True)
            if self.pending_evidence:await self.pending_evidence
            self.stream.close()
            if self.receiver:self.receiver.cancel();await asyncio.gather(self.receiver,return_exceptions=True)
            if self.invalid or self.remote_error:
                self.completed=False;self.failure=self.failure or str(self.remote_error or 'Required observations invalid during final stop audit')
            final=final_stop_status(self.status,(self.latest or {}).get('status'))
            if self.completed and final.get('stop_acknowledged') is not True:
                self.completed=False;self.failure='Final Stop acknowledgement not confirmed'
            status={k:v for k,v in final.items() if k not in ('session','lease')}
            result=dict(passed=self.completed,failure=self.failure,suite=suite,stage='live',test_fixture=False,
                        checks=self.checks,duration_s=(time.perf_counter_ns()-self.start_ns)/1e9,
                        source_sha256=self.source_manifest,source_changed_during_run=source_hashes()!=self.source_manifest,
                        e6_network=(self.latest or {}).get('stats',{}).get('e6_network'),
                        production_input_sha256=self.fixtures['production_input_sha256'],final_status=status,
                        action_ticks=self.action_ticks,candidate_ticks=self.candidate_ticks,frames=self.frames,invalid=self.invalid,
                        episode=self.episode,stats=(self.latest or {}).get('control',{}).get('stats'),
                        wifi_latency=(self.latest or {}).get('stats',{}).get('wifi_latency'),
                        ssh_tcp_latency=(self.latest or {}).get('stats',{}).get('ssh_tcp_latency'),
                        e6_tcp_latency=(self.latest or {}).get('stats',{}).get('e6_tcp_latency'),
                        synthetic_inputs_only=True,physical_gamepad_acceptance_required=True)
            result['signal_harness_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            result['axis_selection']=getattr(self,'axis_selection',{})
            (self.evidence/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
            (self.evidence/'rpc_timings.json').write_text(json.dumps(self.rpc_timings,indent=2),encoding='utf-8')
            (self.evidence/'protocol_trace.json').write_text(json.dumps(self.protocol_trace,indent=2),encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--suite',choices=('zero','j6','axes','frames-fine','multi','gripper','gripper-cycle','gripper-full-range','record-gamepad','record-keyboard'),required=True)
    p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--shadow-evidence',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--frame-filter',choices=('base','tool'));p.add_argument('--fine-filter',choices=('normal','fine'))
    p.add_argument('--mode-filter',choices=('translation','rotation'));p.add_argument('--axis-filter',type=int,choices=(0,1,2))
    p.add_argument('--gripper-fine',action='store_true',help='Use existing precision-mapped input throughout the independent gripper test')
    a=p.parse_args();fixtures=json.loads(a.fixtures.read_text(encoding='utf-8'))
    axis_cases(a.suite,a.frame_filter,a.fine_filter,a.mode_filter,a.axis_filter)
    harness=LiveSignals(fixtures,a.evidence,a.shadow_evidence)
    harness.axis_selection=dict(frame_filter=a.frame_filter,fine_filter=a.fine_filter,mode_filter=a.mode_filter,axis_filter=a.axis_filter)
    harness.gripper_fine=getattr(a,'gripper_fine',False)
    asyncio.run(harness.run_live(a.suite))
