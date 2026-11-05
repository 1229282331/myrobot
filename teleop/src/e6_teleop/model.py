"""Pinned E6 kinematics and virtual views. No dynamics or hardware backend."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from .math3d import from_rotvec, multiply, inverse, to_rotvec, matrix

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "assets" / "e6"


def numbers(v):
    return " ".join(f"{float(x):.10g}" for x in v)


def rpy_quat(rpy):
    r, p, y = rpy
    return multiply(multiply(from_rotvec([0, 0, y]), from_rotvec([0, p, 0])), from_rotvec([r, 0, 0]))


def look_quat(position, target, up=(0, 0, 1)):
    z = np.asarray(position) - np.asarray(target)
    z = z / np.linalg.norm(z)
    x = np.cross(up, z)
    if np.linalg.norm(x) < .01:
        x = np.cross([0, 1, 0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.column_stack([x, y, z]).ravel())
    return q


class E6Model:
    def __init__(self, render=False):
        manifest = json.loads((ASSETS / "manifest.json").read_text(encoding="utf-8"))
        for entry in manifest["files"]:
            if hashlib.sha256((ASSETS / entry["path"]).read_bytes()).hexdigest() != entry["sha256"]:
                raise ValueError(f"Model asset hash mismatch: {entry['path']}")
        self.manifest = manifest
        self.source = ET.parse(ASSETS / "e6.urdf").getroot()
        self.model = mujoco.MjModel.from_xml_string(self._mjcf())
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)
        self.site = self.model.site("tcp").id
        self.limits = self.model.jnt_range.copy()
        # A simulation initial state, copied from the user's configured standby target.
        self.q = np.radians([-66.5019, -60, 105, -90, 90, 9.1974])
        self.gripper = .6
        self.render_enabled = render
        self.renderer = None
        self.set_state(self.q, self.gripper)
        self.model.geom_pos[self.model.geom("goal").id] = self.position + matrix(self.pose()[1]) @ [0, 0, .25]

    def _mjcf(self):
        root = ET.Element("mujoco", model="e6_kinematic")
        ET.SubElement(root, "compiler", angle="radian", meshdir=str(ASSETS))
        ET.SubElement(root, "option", gravity="0 0 0")
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", offwidth="640", offheight="360")
        ET.SubElement(visual, "headlight", ambient=".4 .4 .4", diffuse=".6 .6 .6", specular=".1 .1 .1")
        asset = ET.SubElement(root, "asset")
        for name in ["base_link", *[f"Link{i}" for i in range(1, 7)]]:
            ET.SubElement(asset, "mesh", name=name, file=f"{name}.STL")
        world = ET.SubElement(root, "worldbody")
        ET.SubElement(world, "light", pos="0 -1 2", dir="0 0 -1", diffuse=".8 .8 .8")
        ET.SubElement(world, "geom", type="plane", size="1.5 1.5 .01", rgba=".16 .19 .22 1", contype="0", conaffinity="0")
        ET.SubElement(world, "geom", name="goal", type="sphere", size=".008", pos=".1 -.2 .35", rgba=".9 .65 .22 .85", contype="0", conaffinity="0")
        ET.SubElement(world, "camera", name="overview", pos=".75 -.85 .72", quat=numbers(look_quat([.75, -.85, .72], [0, -.08, .27])), fovy="45")
        base = ET.SubElement(world, "body", name="base_link")
        ET.SubElement(base, "geom", type="mesh", mesh="base_link", rgba=".52 .56 .61 1", contype="0", conaffinity="0")
        parent = base
        for joint in [j for j in self.source.findall("joint") if j.get("type") == "revolute"]:
            origin = joint.find("origin")
            q = rpy_quat([float(v) for v in origin.get("rpy", "0 0 0").split()])
            name = joint.find("child").get("link")
            body = ET.SubElement(parent, "body", name=name, pos=origin.get("xyz"), quat=numbers(np.roll(q, 1)))
            limits = joint.find("limit")
            ET.SubElement(body, "joint", name=joint.get("name"), type="hinge", axis=joint.find("axis").get("xyz"), limited="true", range=f"{limits.get('lower')} {limits.get('upper')}")
            ET.SubElement(body, "geom", type="mesh", mesh=name, rgba=".64 .70 .76 1", contype="0", conaffinity="0")
            parent = body
        ET.SubElement(parent, "site", name="tcp", size=".004", rgba=".3 .95 .8 1")
        for axis, color in [(0, "1 .25 .25 1"), (1, ".3 1 .5 1"), (2, ".35 .6 1 1")]:
            end = [0, 0, 0]; end[axis] = .055
            ET.SubElement(parent, "geom", type="capsule", fromto="0 0 0 " + numbers(end), size=".0017", rgba=color, contype="0", conaffinity="0")
        ET.SubElement(parent, "geom", type="cylinder", pos="0 0 .004", size=".0325 .0035", rgba=".35 .4 .46 1", contype="0", conaffinity="0")
        ET.SubElement(parent, "geom", type="box", pos="0 0 .045", size=".025 .026 .035", rgba=".15 .2 .25 1", contype="0", conaffinity="0")
        for name, sign in [("finger_left", 1), ("finger_right", -1)]:
            ET.SubElement(parent, "geom", name=name, type="box", pos=f"0 {sign*.027} .11", size=".008 .005 .035", rgba=".82 .85 .87 1", contype="0", conaffinity="0")
        pos = [-.045, -.085, .035]
        ET.SubElement(parent, "camera", name="wrist", pos=numbers(pos), quat=numbers(look_quat(pos, [0, 0, .15], up=(0, 1, 0))), fovy="68")
        return ET.tostring(root, encoding="unicode")

    def set_state(self, q, gripper):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all() or np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]):
            raise ValueError("Joint target exceeds model limits")
        self.q = q.copy()
        self.gripper = float(np.clip(gripper, 0, 1))
        self.data.qpos[:] = self.q
        for name, sign in [("finger_left", 1), ("finger_right", -1)]:
            self.model.geom_pos[self.model.geom(name).id, 1] = sign * (.011 + .025*self.gripper)
        mujoco.mj_forward(self.model, self.data)

    def hide_attachments(self):
        """Unknown real gripper opening and unvalidated accessories are not drawn."""
        for name in ('finger_left','finger_right','goal'):
            self.model.geom_rgba[self.model.geom(name).id,3]=0
        # Hide proxy accessory solids while keeping the Link6 axes.
        for i in range(self.model.ngeom):
            if self.model.geom_bodyid[i]==self.model.site_bodyid[self.site] and self.model.geom_type[i] in (mujoco.mjtGeom.mjGEOM_BOX,mujoco.mjtGeom.mjGEOM_CYLINDER):
                self.model.geom_rgba[i,3]=0

    def pose(self, q=None):
        data = self.data if q is None else self.ik_data
        if q is not None:
            data.qpos[:] = q
            mujoco.mj_forward(self.model, data)
        wxyz = np.zeros(4)
        mujoco.mju_mat2Quat(wxyz, data.site_xmat[self.site])
        return data.site_xpos[self.site].copy(), np.roll(wxyz, -1)

    @property
    def position(self):
        return self.pose()[0]

    def solve(self, target_position, target_quaternion):
        target_position = np.asarray(target_position, dtype=float)
        if not np.isfinite(target_position).all():
            raise ValueError("Non-finite IK target")
        q = self.q.copy()
        for _ in range(80):
            position, rotation = self.pose(q)
            ep = target_position - position
            er = to_rotvec(multiply(target_quaternion, inverse(rotation)))
            if np.linalg.norm(ep) < 1e-5 and np.linalg.norm(er) < 1e-4:
                break
            jp, jr = np.zeros((3, 6)), np.zeros((3, 6))
            mujoco.mj_jacSite(self.model, self.ik_data, jp, jr, self.site)
            jac = np.vstack([jp, jr * .15])
            error = np.r_[ep, er * .15]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6*np.eye(6), error)
            dq *= min(1., .08 / max(np.linalg.norm(dq), 1e-12))
            q = np.clip(q + dq, self.limits[:, 0], self.limits[:, 1])
        position, rotation = self.pose(q)
        errors = dict(position_m=float(np.linalg.norm(target_position-position)), rotation_rad=float(np.linalg.norm(to_rotvec(multiply(target_quaternion, inverse(rotation))))))
        if errors["position_m"] > .001 or errors["rotation_rad"] > np.radians(1):
            raise ValueError(f"IK rejected: {errors['position_m']*1000:.2f} mm / {np.degrees(errors['rotation_rad']):.2f} deg")
        if np.max(np.abs(q-self.q)) > .2:
            raise ValueError("IK rejected: discontinuous joint solution")
        return q, errors

    def render(self, camera):
        if not self.render_enabled:
            return None
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=360, width=640)
        self.renderer.update_scene(self.data, camera=camera)
        return self.renderer.render().copy()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
