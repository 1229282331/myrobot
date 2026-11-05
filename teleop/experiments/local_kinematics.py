"""Controller-sampled local approximation for read-only experiments only.

No device IO, URDF alignment, or live-backend integration. An empirical
holdout is evidence about these samples, not a bound on physical TCP error.
"""
from itertools import product
import math
import numpy as np
from e6_observer.control_math import angle, multiply, pose, rotvec, unit


def finite(value, shape):
    value = np.asarray(value, dtype=float)
    if value.shape != shape or not np.isfinite(value).all():
        raise ValueError('Invalid finite array')
    return value


def inverse(q):
    q = unit(q)
    return np.r_[-q[:3], q[3]]


def logarithm(q):
    q = unit(q)
    if q[3] < 0:
        q = -q
    n = np.linalg.norm(q[:3])
    return 2*q[:3] if n < 1e-12 else q[:3]*(2*math.atan2(n, q[3])/n)


POWERS = np.asarray([v for v in product(range(4), repeat=6) if sum(v) <= 3], dtype=int)
DERIVATIVE_POWERS = np.maximum(POWERS[None, :, :]-np.eye(6, dtype=int)[:, None, :], 0)


def features(x):
    x = np.asarray(x, dtype=float)
    # Avoid repeated generic integer-power operations on the small ARM CPU.
    table = np.stack([np.ones_like(x), x, x*x, x*x*x], axis=-2)
    return np.prod(table[..., POWERS, np.arange(6)], axis=-1)


class LocalKinematics:
    experiment_only = True
    eligible_for_hardware_acceptance = False

    def __init__(self, anchor_q, anchor_pose, span_rad, coefficients):
        self.anchor_q = finite(anchor_q, (6,)).copy()
        self.anchor_p, self.anchor_r = pose(finite(anchor_pose, (6,)))
        self.span = float(span_rad)
        if not math.isfinite(self.span) or not 0 < self.span <= math.radians(10):
            raise ValueError('Local joint domain must be at most 10 degrees')
        self.coefficients = finite(coefficients, (len(POWERS), 6)).copy()

    @classmethod
    def fit(cls, joints, poses, anchor_q, anchor_pose, span_rad):
        joints = np.asarray(joints, float)
        poses = finite(poses, joints.shape)
        if joints.ndim != 2 or joints.shape[1] != 6 or len(joints) < 3*len(POWERS):
            raise ValueError('Insufficient independent fitting samples')
        finite(joints, joints.shape)
        model = cls(anchor_q, anchor_pose, span_rad, np.zeros((len(POWERS), 6)))
        x = (joints-model.anchor_q)/model.span
        if np.max(np.abs(x)) > 1+1e-10:
            raise ValueError('Fitting samples exceed declared domain')
        values = []
        for raw in poses:
            p, r = pose(raw)
            relative = multiply(r, inverse(model.anchor_r))
            if angle(r, model.anchor_r) > math.radians(90):
                raise ValueError('Rotation chart too far from local anchor')
            values.append(np.r_[p-model.anchor_p, logarithm(relative)])
        # The exact controller FK anchor is fixed. Do not let a fitted bias
        # create a first-tick joint jump at the preparation pose.
        design = features(x)[:, 1:]
        coefficients, _, rank, singular = np.linalg.lstsq(design, values, rcond=None)
        if rank != len(POWERS)-1 or singular[0]/singular[-1] > 1e5:
            raise ValueError('Fitting samples do not identify the polynomial')
        model.coefficients[1:] = coefficients
        model.fit_condition = float(singular[0]/singular[-1])
        return model

    def check_domain(self, q):
        q = finite(q, (6,))
        if np.max(np.abs(q-self.anchor_q)) > self.span+1e-12:
            raise ValueError('Outside sampled joint domain; extrapolation refused')
        return q

    def _predict(self, q):
        delta = features((q-self.anchor_q)/self.span)@self.coefficients
        return self.anchor_p+delta[:3], multiply(rotvec(delta[3:]), self.anchor_r)

    def fk(self, q):
        return self._predict(self.check_domain(q))

    def jacobian(self, q):
        q = self.check_domain(q)
        x = (q-self.anchor_q)/self.span
        table = np.stack([np.ones_like(x), x, x*x, x*x*x])
        derivatives = np.prod(table[DERIVATIVE_POWERS, np.arange(6)], axis=-1)*POWERS.T
        gradient = derivatives@self.coefficients/self.span
        w = (features(x)@self.coefficients)[3:]
        theta = np.linalg.norm(w)
        skew = np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
        if theta < 1e-6:
            left = np.eye(3)+.5*skew+skew@skew/6
        else:
            left = np.eye(3)+(1-math.cos(theta))/theta**2*skew+(theta-math.sin(theta))/theta**3*(skew@skew)
        # Relative rotation chart -> base-frame angular velocity, not Euler
        # rates. This retains the complete polynomial geometry and SO(3).
        return np.vstack([gradient[:,:3].T, left@gradient[:,3:].T])

    def solve(self, target_p, target_r, seed):
        target_p = finite(target_p, (3,)); target_r = unit(target_r)
        q = self.check_domain(seed).copy(); iterations = 0
        for iterations in range(20):
            p, r = self.fk(q)
            error = np.r_[target_p-p, logarithm(multiply(target_r, inverse(r)))]
            if np.linalg.norm(error[:3]) <= 1e-8 and np.linalg.norm(error[3:]) <= 1e-7:
                break
            jac = self.jacobian(q); jac[3:] *= .15
            singular = np.linalg.svd(jac, compute_uv=False)
            if singular[-1] < 1e-5 or singular[0]/singular[-1] > 1e4:
                raise ValueError('Local Jacobian near singular; target refused')
            weighted = error.copy(); weighted[3:] *= .15
            dq = jac.T@np.linalg.solve(jac@jac.T+1e-8*np.eye(6), weighted)
            dq *= min(1, .05/max(np.linalg.norm(dq), 1e-12))
            accepted = False
            for step in (1, .5, .25, .125, .0625):
                trial = q+step*dq
                try:
                    tp, tr = self.fk(trial)
                except ValueError:
                    continue
                trial_error = np.r_[target_p-tp, logarithm(multiply(target_r, inverse(tr)))*.15]
                if np.linalg.norm(trial_error) < np.linalg.norm(weighted):
                    q = trial; accepted = True; break
            if not accepted:
                raise ValueError('Local solve cannot improve within sampled domain')
        p, r = self.fk(q)
        residual = dict(position_m=float(np.linalg.norm(target_p-p)), rotation_rad=angle(target_r, r),
                        iterations=iterations+1, reference='local approximation, not physical TCP')
        if residual['position_m'] > 1e-8 or residual['rotation_rad'] > 1e-7:
            raise ValueError('Local solve residual exceeds internal tolerance')
        if np.max(np.abs(q-np.asarray(seed))) > math.radians(2):
            raise ValueError('Discontinuous local solution')
        return q, residual

    def serializable(self):
        from e6_observer.control_math import controller_pose
        return dict(schema_version=1, experiment_only=True, eligible_for_hardware_acceptance=False,
                    representation='degree-3 polynomial of controller FK, base-frame relative SE3 chart',
                    anchor_joints_rad=self.anchor_q.tolist(),
                    anchor_pose_mm_deg=controller_pose(self.anchor_p, self.anchor_r, [0]*6),
                    joint_span_rad=self.span, coefficients=self.coefficients.tolist(),
                    polynomial_powers=POWERS.tolist(), fit_condition=self.fit_condition,
                    physical_accuracy_verified=False)
