"""One explicit speed profile for UI intent limits and the upstream solver.

Profiles describe requested capabilities, not qualification or operator release.
Higher profiles require progressive actual motion/stop qualification separately.
"""
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class MotionProfile:
    name: str
    linear_m_s: float
    angular_deg_s: float
    input_joint_deg_s: float
    solver_joint_deg_s: float
    solver_joint_acceleration_deg_s2: float
    singularity_slowdown: float = 17.
    singularity_stop: float = 30.
    gripper_rate_s: float = .025
    input_ramp_time_constant_s: float = 0.

    def input_limits(self):
        from .intent import Limits
        return Limits(linear_m_s=self.linear_m_s, angular_rad_s=math.radians(self.angular_deg_s),
                      joint_rad_s=math.radians(self.input_joint_deg_s), gripper_rate_s=self.gripper_rate_s)

    def joint_limits(self):
        return dict(has_velocity_limits=True, max_velocity=math.radians(self.solver_joint_deg_s),
                    has_acceleration_limits=True,
                    max_acceleration=math.radians(self.solver_joint_acceleration_deg_s2),
                    has_jerk_limits=True, max_jerk=math.radians(10*self.solver_joint_acceleration_deg_s2))

    def description(self):
        return dict(asdict(self), qualification='candidate; actual hardware qualification is separate')


PROFILES = {
    'commissioning': MotionProfile('commissioning', .002, 1, 1, 2, 4),
    # Candidate E6 settings: the measured nominal Jacobian condition is 22.30
    # while requested 30 mm/s needs only 13.41 deg/s. Generic 17/30 defaults
    # halve this nominal command before the 30 deg/s joint bound applies.
    # Keep the commissioning profile unchanged pending real qualification.
    'intermediate': MotionProfile('intermediate', .010, 5, 5, 15, 30, 30., 60., .05),
    # Critically damped intent shaping before IK: gradual startup/reversal,
    # unchanged steady speed and TCP path. Official joint acceleration stays
    # 30 deg/s^2. Release bypasses the filter and clears all of its state.
    'normal': MotionProfile('normal', .030, 10, 10, 30, 30, 30., 60., .1,
                            input_ramp_time_constant_s=.2),
}


def get_profile(name='commissioning'):
    try:
        return PROFILES[name]
    except (KeyError, TypeError):
        raise ValueError('Unknown motion profile: '+str(name)) from None
