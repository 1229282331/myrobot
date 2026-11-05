"""Fetch only pinned public model assets; never execute downloaded code."""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
COMMIT = "ec201c40d5db63185a3593ec9c39f80fe25aa527"
BASE = f"https://raw.githubusercontent.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/{COMMIT}/"


def main():
    dest = ROOT / "assets" / "e6"
    dest.mkdir(parents=True, exist_ok=True)
    sources = []
    original = ROOT.parent / "research" / "sources" / "me6_robot_ec201c40.urdf"
    data = original.read_bytes()
    (dest / "e6.urdf").write_bytes(data)
    sources.append(dict(path="e6.urdf", source=str(original.relative_to(ROOT.parent)),
                        sha256=hashlib.sha256(data).hexdigest()))
    for name in ["base_link", *[f"Link{i}" for i in range(1, 7)]]:
        url = BASE + f"dobot_rviz/meshes/me6/{name}.STL"
        data = urllib.request.urlopen(url, timeout=30).read()
        (dest / f"{name}.STL").write_bytes(data)
        sources.append(dict(path=f"{name}.STL", url=url, sha256=hashlib.sha256(data).hexdigest()))
    url = BASE + "LICENSE"
    data = urllib.request.urlopen(url, timeout=30).read()
    (dest / "LICENSE").write_bytes(data)
    sources.append(dict(path="LICENSE", url=url, sha256=hashlib.sha256(data).hexdigest()))
    (dest / "manifest.json").write_text(json.dumps(dict(commit=COMMIT, files=sources), indent=2) + "\n", encoding="utf-8")
    print(f"Pinned model: {len(sources)} assets, hashes recorded")


if __name__ == "__main__":
    main()
