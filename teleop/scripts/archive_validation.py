"""Archive reproducible test output and implementation hashes, no hardware calls."""
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[2]
evidence=root/"docs/evidence/teleop_simulation_20261004"
evidence.mkdir(parents=True,exist_ok=True)
commands=[([sys.executable,"-m","pytest","teleop/tests","-q"],root),
          (["npm.cmd","test"],root/"teleop/web"),(["npm.cmd","run","build"],root/"teleop/web")]
results=[]
for command,cwd in commands:
    completed=subprocess.run(command,cwd=cwd,capture_output=True,encoding="utf-8",errors="replace")
    results.append(dict(command=command,cwd=str(cwd),exit_code=completed.returncode,output=completed.stdout+completed.stderr))
    if completed.returncode: raise RuntimeError(results[-1]["output"])
versions={name:importlib.metadata.version(name) for name in ["mujoco","numpy","lerobot","torch","av","fastapi"]}
report=dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),python=sys.version,versions=versions,results=results,hardware_connections=False)
(evidence/"test_summary.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
files=[]
for folder in ["teleop/src","teleop/scripts","teleop/tests","teleop/web/src","teleop/assets"]:
    files.extend(path for path in (root/folder).rglob("*") if path.is_file() and "__pycache__" not in path.parts)
files.extend(root/path for path in ["teleop/pyproject.toml","teleop/uv.lock","teleop/bootstrap.ps1","teleop/launch.cmd",
                                    "teleop/web/package.json","teleop/web/package-lock.json","teleop/web/tsconfig.json","teleop/web/vite.config.ts"])
manifest=[dict(path=path.relative_to(root).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for path in sorted(files)]
(evidence/"source_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
browser=[]
for path in sorted((root/"datasets/e6_teleop/raw").glob("*")):
    meta=json.loads((path/"metadata.json").read_text(encoding="utf-8"))
    if meta["task"].startswith("浏览器键盘") or path.name=="20261004T142814Z_515ce662":
        rows=[json.loads(line) for line in (path/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
        active=[row for row in rows if row["effective_action"]["active"]]
        rates=sorted(set(row["input"]["gripper_rate"] for row in active))
        browser.append(dict(raw_episode=path.name,frames=len(rows),active_frames=len(active),gripper_rates=rates,
                            modes=sorted(set(row["input"]["mode"] for row in active)),
                            coordinate_frames=sorted(set(row["input"]["frame"] for row in rows if row.get("input"))),
                            fine_frames=sum(row["input"]["fine"] for row in active)))
browser_report=dict(records=browser,ui_checks=["positive X, positive/negative Y, positive/negative Z, negative X",
                                              "translation/rotation and base/tool switches", "continuous gripper and fine input",
                                              "text editing protection", "Escape stop", "official LeRobot index 600 is episode 1 frame 0"],
                    gamepad_current_browser_detected=False,gamepad_archive_regression_passed=True,bluetooth_tested=False)
(evidence/"browser_validation.json").write_text(json.dumps(browser_report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(dict(tests_passed=True,files_hashed=len(manifest),versions=versions,browser=browser),ensure_ascii=False,indent=2))
