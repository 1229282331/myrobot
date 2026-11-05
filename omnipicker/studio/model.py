"""Official URDF assembly and an explicitly nominal, unloaded four-bar linkage.

No joint angle or position telemetry is inferred from the device. The passive
third joint is held at its URDF zero; the loop endpoint is closed geometrically.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ASSET = Path(__file__).resolve().parents[1] / "assets/official/robot_description"


def vec(text):
    return np.array([float(x) for x in text.split()])


def rotation(axis, angle):
    x, y, z = np.asarray(axis) / np.linalg.norm(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[t*x*x+c,t*x*y-s*z,t*x*z+s*y],
                     [t*x*y+s*z,t*y*y+c,t*y*z-s*x],
                     [t*x*z-s*y,t*y*z+s*x,t*z*z+c]])


def origin(node):
    m = np.eye(4)
    if node is not None:
        m[:3, 3] = vec(node.get("xyz", "0 0 0"))
        r,p,y = vec(node.get("rpy", "0 0 0"))
        m[:3,:3] = rotation([0,0,1], y) @ rotation([0,1,0], p) @ rotation([1,0,0], r)
    return m


class Assembly:
    def __init__(self):
        root = ET.parse(ASSET / "urdf/robot_urdf.xacro").getroot()
        self.joints = []
        self.visuals = []
        self.links = [x.get("name") for x in root.findall("link")]
        for j in root.findall("joint"):
            self.joints.append(dict(name=j.get("name"), parent=j.find("parent").get("link"),
                child=j.find("child").get("link"), matrix=origin(j.find("origin")),
                axis=vec(j.find("axis").get("xyz")), fixed=j.get("type")=="fixed"))
        for link in root.findall("link"):
            v = link.find("visual")
            if v is not None:
                filename = v.find("geometry/mesh").get("filename").split("/")[-1]
                self.visuals.append((link.get("name"), ASSET / "meshes" / filename, origin(v.find("origin"))))
        self.by_name = {j["name"]: j for j in self.joints}

    def nominal_angles(self, percent):
        if not math.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError("Invalid nominal opening")
        result = {}
        for side, sign in (("narrow",1), ("wide",-1)):
            prefix = "hand_" + side
            root = self.by_name[prefix+"1_joint"]["matrix"]
            loop = self.by_name[prefix+"_loop_joint"]["matrix"]
            b = self.by_name[prefix+"2_joint"]["matrix"][:2,3]
            c = self.by_name[prefix+"3_joint"]["matrix"][:2,3]
            e = self.by_name[prefix+"4_joint"]["matrix"][:2,3]
            d = (np.linalg.inv(root) @ loop)[:2,3]
            e0 = b + c + e
            # Display-only endpoint range. Not a calibrated physical stroke.
            q1 = sign * math.radians(-45 * percent / 100)
            b_now = (rotation([0,0,-1],q1)[:2,:2] @ b)
            be, de = np.linalg.norm(c+e), np.linalg.norm(e0-d)
            delta = d-b_now
            distance = np.linalg.norm(delta)
            along = (be*be-de*de+distance*distance)/(2*distance)
            height2 = be*be-along*along
            if height2 < -1e-12: raise ValueError("Nominal linkage outside closure range")
            unit = delta/distance
            center = b_now+along*unit
            offset = math.sqrt(max(0.,height2))*np.array([-unit[1],unit[0]])
            # The zero-pose assembly branch is stable throughout the chosen arc.
            points = (center+offset, center-offset)
            end = min(points, key=lambda p: np.linalg.norm(p-e0))
            angle = lambda p: math.atan2(p[1],p[0])
            wrap = lambda a: math.atan2(math.sin(a),math.cos(a))
            result[prefix+"1_joint"] = q1
            result[prefix+"2_joint"] = wrap(-(angle(end-b_now)-angle(c+e))-q1)
            result[prefix+"3_joint"] = 0.
            result[prefix+"_loop_joint"] = wrap(-(angle(end-d)-angle(e0-d)))
        return result

    def transforms(self, angles):
        matrices = {"base_link":np.eye(4)}
        for j in self.joints:
            joint_rotation = np.eye(4)
            if not j["fixed"]:
                joint_rotation[:3,:3] = rotation(j["axis"], angles.get(j["name"],0.))
            matrices[j["child"]] = matrices[j["parent"]] @ j["matrix"] @ joint_rotation
        return matrices


def stl_vertices(path):
    raw = Path(path).read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84+count*50: raise ValueError("Invalid binary STL: " + str(path))
    triangles = np.frombuffer(raw, dtype=np.dtype([("n","<f4",3),("v","<f4",(3,3)),("attr","<u2")]), offset=84, count=count)
    vertices = triangles["v"].reshape(-1,3).copy() * 1000
    # Recompute normals to avoid malformed exported normals.
    cross = np.cross(triangles["v"][:,1]-triangles["v"][:,0], triangles["v"][:,2]-triangles["v"][:,0])
    norm = np.linalg.norm(cross, axis=1, keepdims=True)
    normals = np.repeat(cross/np.maximum(norm,1e-20),3,axis=0)
    return np.concatenate([vertices,normals],axis=1).astype("<f4")
