"""Timing/provenance for the official C++ solver. No hardware or ROS imports.

This is an endpoint adapter, not an input lease or motion owner. The gateway
continues to own input authorization, independent feedback and immediate Stop.
"""
from copy import deepcopy
import math
import numpy as np
from scipy.spatial.transform import Rotation


def fresh_actual(actual, now):
    stamp = actual.get('received_ns')
    return bool(actual.get('connected') and actual.get('status') and
                type(stamp) is int and 0 <= now - stamp <= 100_000_000)


def finite_vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('Invalid official solver vector')
    return result


def model_error(pose, actual):
    observed = actual['status']['tool_vector_actual']
    xyz = finite_vector(pose['position_m'], 3)
    quat = finite_vector(pose['quaternion_xyzw'], 4)
    if abs(np.linalg.norm(quat) - 1) > 1e-6:
        raise ValueError('Invalid official solver quaternion')
    expected_xyz = finite_vector([observed[k] for k in ('x', 'y', 'z')], 3)
    expected_rotation = Rotation.from_euler('xyz', [observed[k] for k in ('rx', 'ry', 'rz')], degrees=True)
    return dict(position_mm=float(np.linalg.norm(xyz * 1000 - expected_xyz)),
                orientation_deg=math.degrees((Rotation.from_quat(quat).inv() * expected_rotation).magnitude()))


class CppCycle:
    period_ns = 50_000_000

    def __init__(self,tcp_profile='flange',input_ramp_time_constant_s=0.,dynamic_gripper=False,orientation_assist=False,writer_paced=False,
                 *,callback_budget_ms=42,request_timeout_ms=50,delivery_timing=None):
        # Computation may occasionally outlast one integration step. Its
        # deadline is separate from the nominal step and sole-writer pacing.
        # The worker must configure/attest the matching native callback budget.
        if (type(callback_budget_ms) is not int or type(request_timeout_ms) is not int or
                not 0 < callback_budget_ms < request_timeout_ms <= 100):
            raise ValueError('Invalid solver timing; request must fit source feedback lifetime')
        self.callback_limit_us=callback_budget_ms*1000
        self.request_timeout_ns=request_timeout_ms*1_000_000
        self.timing=dict(callback_budget_ms=callback_budget_ms,request_timeout_ms=request_timeout_ms)
        self.writer_paced=writer_paced
        from .delivery_timing import validate_delivery,source_lifetime_ns
        self.delivery=validate_delivery(delivery_timing) if delivery_timing is not None else None
        if self.delivery is not None and not writer_paced:
            raise ValueError('Delivery timing requires writer pacing')
        self.source_lifetime_ns=source_lifetime_ns(self.delivery)
        self.orientation_assist=orientation_assist
        self.assist_anchor=None
        self.dynamic_gripper=dynamic_gripper
        from .tool_frame import get_tool
        self.tool=get_tool(tcp_profile)
        from .intent_ramp import IntentRamp
        self.ramp=IntentRamp(input_ramp_time_constant_s) if input_ramp_time_constant_s else None
        self.context = None
        self.pending = None
        self.counter = 0
        self.next_request_ns = 0
        self.last_active_request_ns = None
        self.initialized = False
        self.reset_required = True
        self.generation = None
        self.previous_candidate = None
        self.first_epoch = None
        self.first_computed_ns = None
        self.latest = None
        self.rejections = {}
        self.blocked_epoch = None
        self.info = dict(ready=False, phase='starting', status=None, error=None,
                         candidate_selection='official_cpp_step', solver_period_s=.05,
                         timing=dict(self.timing))
        if writer_paced:self.info['writer_pacing_version']=1
        if self.delivery is not None:self.info['delivery_timing']=dict(self.delivery)

    def update(self, context):
        self.context = context
        if self.blocked_epoch is not None and context['epoch'] != self.blocked_epoch:
            self.blocked_epoch = None
        if (self.latest and (not context.get('command') or
                            self.latest['epoch'] != context['epoch'])):
            self.latest = None

    def active(self, now):
        if not self.context or not fresh_actual(self.context['actual'], now):
            return False
        return self.input_active(now)

    def input_active(self, now):
        if not self.context:
            return False
        command = self.context.get('command')
        return bool(command and command['epoch'] == self.context['epoch'] and
                    0 <= now - command['received_ns'] <= 250_000_000 and
                    command['intent']['deadman'] and
                    any(command['intent']['joint_rad_s'] + command['intent']['linear_m_s'] +
                        command['intent']['angular_rad_s']))

    def reject(self, reason, detail=None):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        self.latest = None
        self.reset_required = True
        if self.ramp:self.ramp.reset()
        self.info.update(last_rejection=reason, rejections=dict(self.rejections),
                         last_rejection_detail=detail)

    def reject_reply(self, reason, pending, reply, now):
        # Keep one bounded, immutable failure record across neutral/reset and
        # successful replies. The latest-state IPC can otherwise coalesce the
        # rejected reply with a reset and hide why the candidate stream stopped.
        self.reject(reason, dict(reason=reason, id=pending['id'], epoch=pending['epoch'],
            started_ns=pending['started_ns'], at_ns=now,
            actual_received_ns=pending['actual']['received_ns'],
            input_received_ns=pending['command']['received_ns'],
            input_sequence=pending['command']['sequence'],
            deadline_ns=min(pending['actual']['received_ns']+self.source_lifetime_ns,
                            pending['command']['received_ns']+250_000_000),
            roundtrip_ms=(now-pending['started_ns'])/1e6,
            timing=dict(self.timing),
            status=reply['status'], callback_us=reply['callback_us'], compute_us=reply['compute_us'],
            q_rad=list(pending['q']), effective_intent=deepcopy(pending['effective_intent']),
            gripper_check=deepcopy(reply.get('gripper_check'))))
        if reason != 'released_or_expired_input':
            self.blocked_epoch = pending['epoch']

    def request(self, now):
        if self.pending is not None or self.context is None:
            return None
        actual = self.context['actual']
        if not fresh_actual(actual, now):
            if self.delivery is not None:
                # A delayed IPC context is not permission to reseed native
                # state from feedback. The gateway independently owns the
                # live feedback watchdog. Wait without advancing/resetting;
                # its stream deadline or a fresh context resolves this wait.
                self.info['context_waiting']=True
                return None
            self.reject('feedback_stale')
            self.info.update(ready=False, phase='paused', error='Feedback unavailable')
            return None
        if self.delivery is not None:self.info['context_waiting']=False
        command = self.context.get('command')
        active = self.active(now) and self.initialized and not self.reset_required
        if self.blocked_epoch == self.context['epoch']:
            # Do not manufacture an inactive reset and resume a held input.
            # The immutable rejection remains visible until the owner cancels
            # this epoch. Gateway still requires a fresh operator neutral.
            active = False
        generation = actual.get('generation')
        if generation != self.generation:
            active = False
            self.reset_required = True
        epoch = self.context['epoch']
        # A new active epoch can start promptly; subsequent calculations are
        # fixed 20 Hz. Neutral/release also bypasses the calculation schedule.
        new_epoch = active and (not self.previous_candidate or self.previous_candidate['epoch'] != epoch)
        releasing = not active and self.previous_candidate is not None
        dispatch=None
        if active and self.writer_paced:
            control=self.context.get('dispatch') or {}
            if control.get('version')!=1:
                raise ValueError('Missing sole-writer pacing protocol')
            receipt=control.get('receipt')
            previous=self.previous_candidate
            if previous and previous['epoch']==epoch:
                # The native smoother has computed this point, but cannot
                # advance past it until the sole writer actually sends it.
                if not receipt or (receipt.get('id'),receipt.get('epoch'))!=(previous['id'],epoch):
                    return None
                if (type(receipt.get('sent_ns')) is not int or not 0<=receipt['sent_ns']<=now or
                        not np.allclose(finite_vector(receipt.get('joints_rad'),6),previous['q'],atol=1e-12,rtol=0)):
                    raise ValueError('Writer receipt does not match solver parent')
            dispatch=dict(version=1,not_before_ns=(receipt['sent_ns']+self.period_ns if receipt else now),
                          parent_sent_ns=receipt['sent_ns'] if receipt else None)
        if now < self.next_request_ns and not new_epoch and not releasing and not (active and self.writer_paced):
            return None
        # The producer is the sole motion clock, including rapid new epochs.
        # Releases/reset bypass it; re-pressing cannot create a compute burst.
        if active and not self.writer_paced and self.last_active_request_ns is not None and now-self.last_active_request_ns<self.period_ns:
            return None
        q = np.deg2rad(finite_vector(actual['status']['joint_state']['q_actual'], 6))
        intent = command['intent'] if active else None
        effective=intent;workspace_limit=None;input_ramp=None
        if self.ramp:
            if active:effective,input_ramp=self.ramp.step(intent,epoch)
            else:self.ramp.reset()
        if active and command.get('workspace'):
            from .workspace_velocity import damp_translation
            previous=self.previous_candidate
            if previous and previous['epoch']==epoch:
                position=np.array(previous['tcp_position_m'])
                rotation=Rotation.from_quat(previous['tcp_quaternion_xyzw'])
                if effective.get('orientation_assist') and previous.get('assist_anchor') is not None:
                    rotation=Rotation.from_quat(previous['assist_anchor'])
            else:
                p=actual['status']['tool_vector_actual']
                position,rotation=self.tool.apply(np.array([p[k] for k in ('x','y','z')])/1000,
                    Rotation.from_euler('xyz',[p[k] for k in ('rx','ry','rz')],degrees=True))
            effective,workspace_limit=damp_translation(effective,position,rotation,command['workspace'])
        assisted=bool(intent and intent.get('orientation_assist'))
        if assisted and not self.orientation_assist:
            self.reject('orientation_assist_unavailable')
            return None
        kind = 'a' if assisted else 'j' if intent and any(intent['joint_rad_s']) else 't'
        values = ([0.] * 6 if not active else effective['joint_rad_s'] if kind == 'j'
                  else effective['linear_m_s'] + effective['angular_rad_s'])
        finite_vector(values, 6)
        tool = bool(intent and intent['frame'] == 'tool')
        self.counter += 1
        self.pending = dict(id=self.counter, epoch=epoch, active=active, kind=kind,
                            started_ns=now, actual=deepcopy(actual), command=deepcopy(command),
                            q=q.tolist(), generation=generation, tool=tool,
                            effective_intent=deepcopy(effective),workspace_limit=workspace_limit,input_ramp=input_ramp,
                            previous=deepcopy(self.previous_candidate),dispatch=dispatch)
        self.next_request_ns = now + self.period_ns
        if active:self.last_active_request_ns=now
        extra=['A1',intent['assist_angular_rad_s']] if assisted else []
        if self.dynamic_gripper:
            from .gripper_geometry import request_state,retreat_hold
            counts=request_state(self.context.get('gripper',{}),now)
            self.pending['gripper_counts']=counts
            self.pending['gripper_hold']=retreat_hold(self.context.get('gripper',{}),now)
            extra+=['G3',*counts,self.pending['gripper_hold']]
        return ' '.join(map(str, [self.counter, epoch, int(active), {'j':0,'t':1,'a':2}[kind], int(tool),
                                 *q.tolist(), *values,*extra])) + '\n'

    def complete(self, reply, now):
        pending = self.pending
        if pending is None:
            raise ValueError('Unrequested official solver response')
        self.pending = None
        if (reply.get('seq'), reply.get('epoch'), reply.get('active')) != (
                pending['id'], pending['epoch'], pending['active']):
            raise ValueError('Official solver response identity mismatch')
        q = finite_vector(reply['joints_rad'], 6)
        if type(reply.get('status')) is not int or reply['status'] not in (-1,0,1,2,3,4,5,6):
            raise ValueError('Invalid official status')
        velocity = finite_vector(reply['velocities_rad_s'], 6)
        seed_q = finite_vector(reply['seed_positions'], 6)
        seed_v = finite_vector(reply['seed_velocities'], 6)
        if type(reply.get('reset')) is not bool:
            raise ValueError('Missing smoother reset evidence')
        error = model_error(reply['actual_fk'], pending['actual'])
        aligned = error['position_mm'] <= 1 and error['orientation_deg'] <= 1
        self.initialized = True
        self.generation = pending['generation']
        self.info.update(model_error=error, status=reply['status'],
                         ready=aligned, phase='active' if pending['active'] else 'paused',
                         error=None if aligned else 'Model/feedback not aligned',
                         roundtrip_ms=(now - pending['started_ns']) / 1e6,
                         workspace_limit=pending['workspace_limit'],
                         compute_us=reply['compute_us'], callback_us=reply['callback_us'])
        self.info['singularity']=deepcopy(reply.get('singularity'))
        self.info['collision_retreat']=deepcopy(reply.get('collision_retreat'))
        assist=deepcopy(reply.get('orientation_assist'))
        if self.orientation_assist:
            if (not isinstance(assist,dict) or assist.get('version')!=1 or
                type(assist.get('enabled')) is not bool or assist['enabled']!=(pending['kind']=='a')):
                raise ValueError('Orientation assist response provenance mismatch')
            angular=finite_vector(assist.get('twist_base'),6)[3:]
            anchor=finite_vector(assist.get('anchor_xyzw'),4)
            if abs(np.linalg.norm(anchor)-1)>1e-6:raise ValueError('Invalid assist anchor')
            if assist['enabled']:
                rate=pending['effective_intent']['assist_angular_rad_s']
                deviation=assist.get('deviation_deg')
                if (assist.get('max_angle_deg')!=15 or assist.get('max_rate_rad_s')!=rate or
                    not isinstance(deviation,(int,float)) or not math.isfinite(deviation) or
                    not 0<=deviation or (reply['status'] in (0,1,3,4) and deviation>15+1e-6) or
                    np.linalg.norm(angular)>rate+1e-9):
                    raise ValueError('Orientation assist envelope mismatch')
                rotation=Rotation.from_quat(anchor)
                expected=np.array(pending['effective_intent']['linear_m_s'])
                if pending['tool']:expected=rotation.apply(expected)
                if not np.allclose(assist['twist_base'][:3],expected,atol=1e-10,rtol=0):
                    raise ValueError('Orientation assist changed translation direction')
                if self.assist_anchor and self.assist_anchor[0]==pending['epoch']:
                    if (Rotation.from_quat(self.assist_anchor[1]).inv()*rotation).magnitude()>1e-8:
                        raise ValueError('Orientation assist silently reset its anchor')
                else:
                    _,actual_rotation=self.tool.apply(np.array(reply['actual_fk']['position_m']),
                        Rotation.from_quat(reply['actual_fk']['quaternion_xyzw']))
                    if (actual_rotation.inv()*rotation).magnitude()>1e-8:
                        raise ValueError('Orientation assist anchor is not actual feedback')
                    self.assist_anchor=(pending['epoch'],anchor.tolist())
            self.info['orientation_assist']=assist
        if self.dynamic_gripper:
            check=deepcopy(reply.get('gripper_check'))
            keys=('actual_raw','sent_raw','inflight_raw','requested_raw')
            if not isinstance(check,dict) or type(check.get('hold_state')) is not int or check['hold_state']!=pending['gripper_hold']:
                raise ValueError('Gripper hold response provenance mismatch')
            if (not isinstance(check,dict) or any(type(check.get(k)) is not int for k in keys) or [check.get(k) for k in keys]!=pending['gripper_counts']):
                raise ValueError('Gripper collision response provenance mismatch')
            for key in ('allowed','arm_safe','fallback'):
                if type(check.get(key)) is not bool:raise ValueError('Invalid gripper collision flag')
            for key in ('range_raw','arm_range_raw'):
                limits=check.get(key)
                if not isinstance(limits,list) or len(limits)!=2 or any(type(v) is not int or not -1<=v<=255 for v in limits) or limits[0]>limits[1]:raise ValueError('Invalid gripper sweep range')
            counts=pending['gripper_counts']
            if check['fallback']!=(counts[0]<0):raise ValueError('Gripper fallback provenance mismatch')
            if counts[0]>=0:
                for key,flag,covered in [('range_raw','allowed',counts),('arm_range_raw','arm_safe',counts[:3])]:
                    if check[flag] and any(v>=0 and not check[key][0]<=v<=check[key][1] for v in covered):raise ValueError('Incomplete gripper sweep coverage')
            if not np.allclose(finite_vector(check.get('arm_q_rad'),6),pending['q'],atol=1e-12,rtol=0):raise ValueError('Gripper arm provenance mismatch')
            checked_target=finite_vector(check.get('arm_target_rad'),6)
            if check['arm_safe'] and not np.allclose(checked_target,q,atol=1e-12,rtol=0):raise ValueError('Unchecked arm endpoint')
            check.update(checked_ns=pending['started_ns'])
            if (not aligned or now-pending['started_ns']>=100_000_000 or
                    not 0 <= reply['callback_us'] < self.callback_limit_us):check['allowed']=False
            self.info['gripper_check']=check
        from .margin_recovery import validate_reply
        validate_reply(reply,pending)
        self.info['margin_recovery']=deepcopy(reply.get('margin_recovery'))
        if not pending['active']:
            if not reply['reset'] or np.any(velocity) or not np.allclose(q, pending['q'], atol=1e-12, rtol=0):
                raise ValueError('Neutral response retained an old target')
            self.reset_required = False
            self.previous_candidate = None
            self.latest = None
            if self.blocked_epoch == pending['epoch']:
                self.info['status'] = self.info['last_rejection_detail']['status']
            return None
        authorized=self.input_active(now) if self.delivery is not None else self.active(now)
        if not self.context or self.context['epoch'] != pending['epoch'] or not authorized:
            self.reject_reply('released_or_expired_input',pending,reply,now)
            return None
        if self.context['actual'].get('generation') != pending['generation']:
            self.reject_reply('feedback_generation_changed',pending,reply,now)
            return None
        if not aligned:
            self.reject_reply('model_not_aligned',pending,reply,now)
            return None
        deadline = min(pending['actual']['received_ns'] + self.source_lifetime_ns,
                       pending['command']['received_ns'] + 250_000_000)
        # A longer request deadline does not change the integration step or
        # extend the ORIGINAL feedback/input lifetime. One request stays in
        # flight and the writer's parent receipt prevents queued catch-up.
        if now - pending['started_ns'] >= self.request_timeout_ns or now >= deadline:
            self.reject_reply('solver_deadline',pending,reply,now)
            return None
        # Transport slack does not extend the native computation budget.
        if not 0 <= reply['callback_us'] < self.callback_limit_us:
            self.reject_reply('solver_callback_deadline',pending,reply,now)
            return None
        status = reply['status']
        if status not in (0, 1, 3, 4):
            self.reject_reply('official_solver_blocked',pending,reply,now)
            if status == -1:
                self.info.update(ready=False, error='Official solver rejected command')
            return None
        previous = pending['previous']
        if reply['reset']:
            if not np.allclose(seed_q, pending['q'], atol=1e-12, rtol=0) or np.any(seed_v):
                raise ValueError('Smoother reset source mismatch')
            parent = None
        else:
            if (not previous or previous['epoch'] != pending['epoch'] or
                    not np.allclose(seed_q, previous['q'], atol=1e-12, rtol=0) or
                    not np.allclose(seed_v, previous['v'], atol=1e-12, rtol=0)):
                raise ValueError('Smoother parent state mismatch')
            parent = previous['id']
        target = reply['target_fk']
        finite_vector(target['position_m'], 3)
        quat = finite_vector(target['quaternion_xyzw'], 4)
        if abs(np.linalg.norm(quat) - 1) > 1e-6:
            raise ValueError('Invalid target quaternion')
        if pending['kind']=='a':
            _,target_rotation=self.tool.apply(np.array(target['position_m']),Rotation.from_quat(quat))
            deviation=math.degrees((Rotation.from_quat(assist['anchor_xyzw']).inv()*target_rotation).magnitude())
            if abs(deviation-assist['deviation_deg'])>1e-6 or deviation>15+1e-6:
                raise ValueError('Assisted target exceeds its verified angular envelope')
        command = pending['command']
        source = dict(sequence=command['sequence'], epoch=command['epoch'],
                      at_ns=pending['started_ns'],
                      received_ns=command['received_ns'], intent=command['intent'],
                      effective_intent=pending['effective_intent'],workspace_limit=pending['workspace_limit'],
                      input_ramp=pending['input_ramp'],
                      observation_received_ns=pending['actual']['received_ns'])
        if self.first_epoch != pending['epoch']:
            self.first_epoch = pending['epoch']
            self.first_computed_ns = now
        row = dict(id=pending['id'], epoch=pending['epoch'], received_ns=pending['started_ns'],
                   valid_until_ns=deadline, computed_ns=now, epoch_first_computed_ns=self.first_computed_ns,
                   joints_rad=q.tolist(), position_m=target['position_m'], quaternion_xyzw=quat.tolist(),
                   status=status, input_sequence=source['sequence'], source_input=source,
                   trajectory_sources=[], input_association='official_cpp_step_with_smoother_seed',
                   pre_action_received_ns=pending['actual']['received_ns'],
                   cpp=dict(period_s=.05, timing=dict(self.timing),parent_id=parent, reset=reply['reset'],
                            actual_q_rad=pending['q'], seed_positions=seed_q.tolist(),
                            seed_velocities=seed_v.tolist(), target_velocities=velocity.tolist(),
                             generation=pending['generation'], kind=pending['kind']))
        row['cpp']['singularity']=deepcopy(reply.get('singularity'))
        if self.writer_paced:row['dispatch']=deepcopy(pending['dispatch'])
        if self.delivery is not None:row['cpp']['delivery_timing']=dict(self.delivery)
        row['cpp']['collision_retreat']=deepcopy(reply.get('collision_retreat'))
        row['cpp']['margin_recovery']=deepcopy(reply.get('margin_recovery'))
        if self.orientation_assist:row['cpp']['orientation_assist']=deepcopy(assist)
        if self.dynamic_gripper:row['cpp']['gripper_check']=deepcopy(self.info['gripper_check'])
        tcp_position,tcp_rotation=self.tool.apply(np.array(target['position_m']),Rotation.from_quat(quat))
        self.previous_candidate = dict(id=row['id'], epoch=row['epoch'], q=q.tolist(), v=velocity.tolist(),
                                       tcp_position_m=tcp_position.tolist(),tcp_quaternion_xyzw=tcp_rotation.as_quat().tolist())
        if pending['kind']=='a':self.previous_candidate['assist_anchor']=assist['anchor_xyzw']
        self.latest = row
        return row

    def snapshot(self, now):
        fresh = bool(self.context and fresh_actual(self.context['actual'], now))
        candidate = self.latest
        # With writer pacing an unseen endpoint blocks its successor until a
        # receipt arrives. Deliver its original deadline even when expired so
        # the consumer can cancel that epoch, rather than silently hiding the
        # endpoint and waiting for a receipt that can never exist.
        authorized=self.input_active(now) if self.delivery is not None else self.active(now)
        if not authorized or not candidate or (not self.writer_paced and now >= candidate['valid_until_ns']):
            candidate = None
        # With explicit delivery slack, native readiness and transport
        # freshness are separate. Never calculate on an old context; an
        # already checked point keeps only its original bounded deadline.
        # The sole writer still checks independent, current device feedback.
        ready=bool(self.info['ready'] and (fresh or self.delivery is not None))
        info=dict(self.info,ready=ready)
        if self.delivery is not None:info['context_waiting']=not fresh
        return dict(info=dict(info,
                              shared_clock='same X5 kernel CLOCK_MONOTONIC'), candidate=candidate)
