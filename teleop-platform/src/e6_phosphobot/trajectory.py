"""Ruckig velocity-interface adapter; SI units, no inverse kinematics.

Uses the upstream velocity/stop examples, not a local trajectory solver.
Cartesian channels are base-frame translation and rotation-vector increments.
Only accepted stream steps advance the reference. Late cycles are discarded.
"""
import math
import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, ControlInterface, Synchronization


class VelocityTrajectory:
    def __init__(self, period_s, kind, *, motion_profile=None):
        if kind not in ('cartesian', 'joint'):
            raise ValueError('Unknown trajectory kind')
        self.kind = kind
        self.period_s = period_s
        self.generator = Ruckig(6, period_s)
        self.input = InputParameter(6)
        self.output = OutputParameter(6)
        self.input.control_interface = ControlInterface.Velocity
        self.input.synchronization = Synchronization.Time
        # Initial qualification values. Not claimed as device hardware limits.
        self.input.max_acceleration = ([.02]*3 + [math.radians(10)]*3
                                       if kind == 'cartesian' else [math.radians(10)]*6)
        self.input.max_jerk = ([.2]*3 + [math.radians(100)]*3
                              if kind == 'cartesian' else [math.radians(100)]*6)
        self.motion_profile=motion_profile
        self.last_step=None
        if motion_profile is not None:
            from .motion_profiles import get_profile
            profile=get_profile(motion_profile)
            # Ruckig's second-order velocity interface: time-synchronized
            # acceleration-limited transitions interpolate inside the convex
            # velocity envelope. Finite jerk can overshoot a new velocity
            # while cancelling existing acceleration during rapid turns.
            # These command-space limits do not certify actual joint dynamics.
            velocity=([profile.linear_m_s]*3+[math.radians(profile.angular_deg_s)]*3
                      if kind=='cartesian' else [math.radians(profile.input_joint_deg_s)]*6)
            self.input.max_acceleration=[5*v for v in velocity]
            self.input.max_jerk=[math.inf]*6

    def step(self, velocity):
        values = np.asarray(velocity, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('Six finite velocity channels required')
        previous = np.asarray(self.input.current_position)
        prior_velocity=list(self.input.current_velocity)
        prior_acceleration=list(self.input.current_acceleration)
        self.input.target_velocity = values.tolist()
        self.input.target_acceleration = [0.]*6
        self.generator.validate_input(self.input)
        result = self.generator.update(self.input, self.output)
        if int(result) < 0:
            raise RuntimeError('Ruckig rejected velocity trajectory: '+str(result))
        delta = np.asarray(self.output.new_position) - previous
        self.output.pass_to_input(self.input)
        if not np.isfinite(delta).all():
            raise RuntimeError('Ruckig produced nonfinite target')
        self.last_step=dict(period_s=self.period_s,requested_velocity=values.tolist(),
                            prior_velocity=prior_velocity,prior_acceleration=prior_acceleration,
                            delta=delta.tolist())
        return delta

    @property
    def stopped(self):
        return (max(map(abs, self.input.current_velocity)) < 1e-10 and
                max(map(abs, self.input.current_acceleration)) < 1e-9)

    def state(self):
        return dict(library='ruckig', version='0.19.4', kind=self.kind,
                    motion_profile=self.motion_profile,
                    order='acceleration' if self.motion_profile else 'jerk',
                    step=self.last_step,
                    velocity=self.input.current_velocity,
                    acceleration=self.input.current_acceleration,
                    acceleration_limits=self.input.max_acceleration,
                    jerk_limits=None if self.motion_profile else self.input.max_jerk)
