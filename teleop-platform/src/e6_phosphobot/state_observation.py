"""Render an already captured observation without the control mutex.

This module has no executor, session, input, or device handle. Its result is
display/audit data, never a movement permission or a refreshed feedback lease.
"""
from dataclasses import asdict, dataclass
import pickle

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class StateObservation:
    data: dict
    tool: object
    profile: object
    payload: object
    workspace: object
    limits: object
    captured_ns: int


@dataclass(frozen=True)
class DeferredAuditObservation:
    """Private, immutable journal payload plus cheap captured lifecycle fields."""
    payload: bytes
    fields: dict
    captured_ns: int


def render_observation(observation):
    if isinstance(observation, DeferredAuditObservation):
        # Only locally created bytes on the inherited Audit pipe reach here.
        observation = pickle.loads(observation.payload)
        if not isinstance(observation, StateObservation):
            raise TypeError("Invalid private audit observation")
    state = dict(observation.data)
    sample = state["feedback"]
    gripper = dict(state["gripper"])
    try:
        from .gripper_geometry import pose
        gripper["geometry"] = pose(gripper, observation.captured_ns)
    except (OSError, ValueError, KeyError):
        gripper["geometry"] = None
    state["gripper"] = gripper
    actual = None
    if sample["status"]:
        p = sample["status"]["tool_vector_actual"]
        fp = np.array([p[k] for k in ("x", "y", "z")]) / 1000
        fr = Rotation.from_euler("xyz", [p[k] for k in ("rx", "ry", "rz")], degrees=True)
        position, rotation = observation.tool.apply(fp, fr)
        actual = dict(
            joints_rad=np.deg2rad(sample["status"]["joint_state"]["q_actual"]).tolist(),
            pose_frame=observation.tool.frame,
            flange_pose=dict(position_m=fp.tolist(), quaternion_xyzw=fr.as_quat().tolist(),
                             pose_frame="controller_flange"),
            position_m=position.tolist(), quaternion_xyzw=rotation.as_quat().tolist(),
            gripper=gripper["actual"])
    state["actual"] = actual
    state["capabilities"] = dict(
        state["capabilities"], motion_profile=observation.profile.description(),
        tcp=observation.tool.description(), payload=observation.payload.description(),
        test_workspace=observation.workspace.description(), **asdict(observation.limits))
    return state


def observe_executor(executor, mutex, controller=False):
    """Only capture mutable control fields while holding the input/Stop lock."""
    with mutex:
        observation = executor.capture_state(controller)
    return render_observation(observation)


def capture_audit_observation(executor, mutex):
    """Freeze the sampled state; independent audit processes render it later.

    No executor or connection crosses the private audit pipe. Serialization
    during capture prevents later receipt/session mutations from changing an
    older recorded observation. Its original clock defines aperture validity.
    """
    with mutex:
        observation = executor.capture_state()
        state = observation.data
        fields = {key: state[key] for key in
                  ("state", "ready", "reason", "stationary", "stopping", "stop_uncertain")}
        for key, names in (
            ("feedback", ("connected",)), ("arm", ("payload_matches",)),
            ("gripper", ("phase", "reason", "collision_blocked")),
            ("recording", ("phase", "error")), ("camera_restart", ("phase", "error")),
            ("model", ("status",)),
        ):
            source = state.get(key)
            fields[key] = {name: source.get(name) for name in names} if source else None
        return DeferredAuditObservation(pickle.dumps(observation, protocol=5),
                                        fields, observation.captured_ns)


def observation_fields(value):
    """Read captured lifecycle fields without calculating display geometry."""
    if isinstance(value, DeferredAuditObservation):
        return value.fields
    return value.data if isinstance(value, StateObservation) else value


def render_audit_event(event):
    """Materialize only the private deferred observation fields of a journal."""
    result = dict(event)
    for key in ("state", "final_state"):
        value = result.get(key)
        if isinstance(value, (StateObservation, DeferredAuditObservation)):
            result[key] = render_observation(value)
    return result
