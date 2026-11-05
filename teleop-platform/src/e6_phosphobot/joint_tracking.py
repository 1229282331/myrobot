"""Independent writer/feedback guard. It never generates a recovery target."""
from collections import deque
import math


class JointTrackingGuard:
    # Contact loads can cause a transient following offset. Use one angle
    # allowance for idle rebase and live tracking, with a bounded persistence
    # window for live faults. Candidate continuity/freshness are independent.
    error_limit_deg = 2.0
    persistence_ns = 500_000_000
    rebase_error_deg = error_limit_deg
    sustained_error_deg = error_limit_deg
    sustained_ns = persistence_ns
    progress_error_deg = error_limit_deg
    progress_ns = persistence_ns
    minimum_progress_deg = .02
    # Associate feedback with the recent, physically written trajectory. This
    # is an execution-lag horizon, not an extension of feedback freshness.
    execution_lag_ns = 250_000_000

    def __init__(self, velocity_deg_s, acceleration_deg_s2):
        self.velocity = float(velocity_deg_s)
        self.acceleration = float(acceleration_deg_s2)
        self.last_sent = None
        self.last_sent_ns = None
        self.history = deque()
        self.error_since = [None] * 6
        self.lag_since = [None] * 6
        self.last_sample_ns = None
        self.last_detail = None
        self.sent_history = deque()
        self.rebased_actual = None

    @staticmethod
    def vector(value):
        try:
            if not isinstance(value,(list,tuple)) and getattr(value,'shape',None)!=(6,):
                raise ValueError('Invalid joint vector')
            if len(value) != 6:raise ValueError('Invalid joint vector')
            if any(getattr(x,'ndim',0)!=0 for x in value):raise ValueError('Invalid joint vector')
            result=tuple(float(x) for x in value)
        except (TypeError,ValueError,OverflowError) as exc:
            raise ValueError('Invalid joint vector') from exc
        if not all(math.isfinite(x) for x in result):raise ValueError('Invalid joint vector')
        return result

    @classmethod
    def joints(cls,sample):
        state=sample['status']['joint_state']
        return cls.vector(state['q_actual']),cls.vector(state['q_target'])

    def reject(self, reason, **detail):
        self.last_detail = dict(reason=reason, **detail)
        return reason

    def observe(self, sample, now):
        actual, target = self.joints(sample)
        stamp = sample['received_ns']
        if type(stamp) is not int or not 0 <= now-stamp <= 100_000_000:
            return self.reject('tracking_feedback_stale')
        if self.last_sample_ns is not None and stamp <= self.last_sample_ns:
            return None
        self.last_sample_ns = stamp
        controller_error = tuple(abs(t-a) for t,a in zip(target,actual))
        error = controller_error
        if self.last_sent is not None:
            error = tuple(max(e,abs(q-a)) for e,q,a in zip(error,self.last_sent,actual))
        self.history.append((stamp, actual, error))
        while len(self.history)>1 and self.history[1][0] <= stamp-self.progress_ns:
            self.history.popleft()
        # Latest ServoJ leads the controller's interpolated target. Track
        # execution separately; neither a single loaded frame nor the newest
        # command lead may bypass the 500 ms following-error allowance.
        lag_error = (0.,) * 6
        if self.sent_history:
            cutoff=stamp-self.execution_lag_ns
            while len(self.sent_history)>1 and self.sent_history[1][0]<=cutoff:
                self.sent_history.popleft()
            # Retain the point just before the horizon: piecewise execution
            # between that point and the next remains part of the corridor.
            recent=[q for t,q in self.sent_history if t<=stamp]
            if recent:
                lag_error=tuple(max(min(axis)-a,a-max(axis),0.) for axis,a in zip(zip(*recent),actual))
        for j in range(6):
            if lag_error[j] > self.error_limit_deg:
                if self.lag_since[j] is None:self.lag_since[j]=stamp
                if stamp-self.lag_since[j] >= self.persistence_ns:
                    return self.reject('joint_execution_lag',joint=j+1,
                        error_deg=list(lag_error),horizon_ns=self.execution_lag_ns,
                        limit_deg=self.error_limit_deg,duration_ms=(stamp-self.lag_since[j])/1e6)
            else:self.lag_since[j]=None
        for j in range(6):
            # ServoJ commands lead execution. Comparing the newest write to
            # the current encoder at this threshold confuses ordinary command
            # latency with servo error (v310 recorded reversals). Use the
            # controller target and encoder from the same feedback frame for
            # sustained servo error. Outstanding writes remain covered by the
            # bounded execution-history corridor and no-progress window below.
            if controller_error[j] > self.sustained_error_deg:
                if self.error_since[j] is None:self.error_since[j]=stamp
                if stamp-self.error_since[j] >= self.sustained_ns:
                    return self.reject('joint_tracking_sustained',joint=j+1,
                        error_deg=list(controller_error),command_or_controller_error_deg=list(error),
                        limit_deg=self.sustained_error_deg,duration_ms=(stamp-self.error_since[j])/1e6)
            else:self.error_since[j]=None
        if self.history and stamp-self.history[0][0] >= self.progress_ns:
            positions=zip(*(h[1] for h in self.history))
            errors=zip(*(h[2] for h in self.history))
            for j,(axis,axis_errors) in enumerate(zip(positions,errors)):
                if min(axis_errors)>self.progress_error_deg and max(axis)-min(axis)<self.minimum_progress_deg:
                    return self.reject('joint_no_progress',joint=j+1,error_deg=list(error),
                        limit_deg=self.progress_error_deg,duration_ms=(stamp-self.history[0][0])/1e6)
        return None

    def candidate(self, degrees, sample, now):
        actual, target = self.joints(sample)
        try:q=self.vector(degrees)
        except ValueError:return self.reject('invalid_joint_candidate')
        # A delayed/coalesced result never earns an arbitrarily large jump.
        # Cross-reset checks use the last physical write, not the new IK seed.
        if self.last_sent is None:
            bound=self.acceleration*.05**2+.02
            origin=target if self.rebased_actual is None else self.rebased_actual
            delta=tuple(abs(x-y) for x,y in zip(q,origin))
            if self.rebased_actual is not None:
                # The solver also starts at actual. Do not command the old
                # loaded QTarget, or turn later actual drift into permission.
                delta=tuple(max(d,abs(x-a)) for d,x,a in zip(delta,q,actual))
        else:
            dt=min(.1,max(0.,(now-self.last_sent_ns)/1e9))
            bound=self.velocity*dt+.02
            # This interval is between physical writes, so compare those
            # writes. QTarget is the controller's delayed execution target;
            # its accumulated lag cannot be charged to one write interval.
            # The same-frame error, bounded history corridor and no-progress
            # observation guard still reject execution failure and windup.
            delta=tuple(abs(x-y) for x,y in zip(q,self.last_sent))
        if max(delta)>bound:
            return self.reject('joint_command_discontinuity',joint=delta.index(max(delta))+1,step_deg=list(delta),limit_deg=bound)
        return None

    def committed(self, degrees, sent_ns):
        self.last_sent=self.vector(degrees);self.last_sent_ns=sent_ns
        self.sent_history.append((sent_ns,self.last_sent))

    def rebase(self, sample):
        """Only caller-confirmed idle/Stop may establish a new writer origin."""
        actual,target=self.joints(sample)
        error=tuple(abs(t-a) for t,a in zip(target,actual))
        if max(error)>self.rebase_error_deg:
            return self.reject('joint_target_not_settled',
                               joint=error.index(max(error))+1,
                               error_deg=list(error),limit_deg=self.rebase_error_deg)
        self.last_sent=None;self.last_sent_ns=None;self.history.clear()
        self.rebased_actual=actual
        self.sent_history.clear()
        self.sent_history.append((sample['received_ns'],actual))
        self.error_since=[None]*6;self.lag_since=[None]*6;self.last_sample_ns=None
        return None
