"""Critically damped operator velocity shaping, before official MoveIt IK.

Two identical first-order stages (H(s)=1/(tau*s+1)^2), discretized by
SciPy's exact zero-order hold. Positive unity-gain impulse response: bounded
input vectors remain in their convex velocity envelope, without overshoot.
This is input conditioning, not IK or a joint trajectory/jerk guarantee.
The motion owner handles Stop; inactive/expired epochs discard this state.
"""
from copy import deepcopy
import numpy as np
from scipy.signal import cont2discrete

FIELDS=('linear_m_s','angular_rad_s','joint_rad_s')


class IntentRamp:
    def __init__(self,time_constant_s,period_s=.05):
        if not .05<=time_constant_s<=1 or period_s!=.05:
            raise ValueError('Invalid intent ramp configuration')
        self.tau=float(time_constant_s);self.period=float(period_s)
        rate=1/self.tau
        self.a,self.b,_,_,_=cont2discrete(
            (np.array([[-rate,0],[rate,-rate]]),np.array([[rate],[0]]),
             np.array([[0,1]]),np.array([[0]])),self.period,method='zoh')
        self.reset()

    def reset(self):
        self.key=None
        self.state=np.zeros((2,12))

    def step(self,intent,epoch):
        key=(epoch,intent['frame'],bool(any(intent['joint_rad_s'])),bool(intent.get('orientation_assist')))
        reset=key!=self.key
        if reset:self.reset();self.key=key
        requested=np.asarray([v for k in FIELDS for v in intent[k]],dtype=float)
        if requested.shape!=(12,) or not np.isfinite(requested).all():
            raise ValueError('Invalid velocity input')
        prior=self.state.copy()
        self.state=self.a@prior+self.b@requested[None,:]
        shaped=deepcopy(intent);offset=0
        for field in FIELDS:
            n=len(intent[field]);shaped[field]=self.state[1,offset:offset+n].tolist();offset+=n
        trace=dict(kind='critically_damped_zoh',time_constant_s=self.tau,period_s=self.period,
                   reset=reset,prior_state=prior.tolist(),state=self.state.tolist(),intent=deepcopy(shaped))
        return shaped,trace
