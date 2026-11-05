"""Append-only simulated episodes; raw observations and effective targets remain distinct."""
from datetime import datetime, timezone
import base64
import hashlib
import json
from pathlib import Path
import time
import uuid


class EpisodeRecorder:
    def __init__(self, root):
        self.root = Path(root)
        self.stream = None
        self.directory = None
        self.samples = 0
        self.last_id = None
        self.incomplete = False
        self.latest_episode = None

    @property
    def active(self):
        return self.stream is not None

    def start(self, task, metadata):
        if self.active:
            raise ValueError("已有episode正在录制")
        if not isinstance(task, str) or not 0 < len(task.strip()) <= 500:
            raise ValueError("请输入任务描述（1–500字符）")
        self.root.mkdir(parents=True, exist_ok=True)
        directory = self.root / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8])
        directory.mkdir()
        (directory / "images").mkdir()
        metadata.update(schema_version=1, dataset_kind="simulation", is_real_demonstration=False,
                        fps=20, task=task.strip(), created_at_utc=datetime.now(timezone.utc).isoformat(),
                        action_semantics="effective absolute joint targets [rad] + normalized gripper; observation precedes action",
                        quaternion_order="xyzw", clock_api="time.perf_counter_ns (monotonic)", clock_note="PC and gateway monotonic clocks are separate domains; no cross-host subtraction")
        (directory / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.stream = (directory / "samples.jsonl").open("x", encoding="utf-8", buffering=1)
        self.directory, self.samples, self.last_id, self.incomplete = directory, 0, None, False
        return directory.name

    def sample(self, frame):
        if not self.active:
            return
        frame_id = frame["observation"]["frame_id"]
        if self.last_id is not None and frame_id != self.last_id + 1:
            self.incomplete = True
            raise ValueError("录制观测缺帧，停止本次episode")
        if not frame["observation"]["valid"] or not frame["observation"]["is_simulated"]:
            self.incomplete = True
            raise ValueError("缺少有效模拟观测")
        data = base64.b64decode(frame["images"]["wrist_png"], validate=True)
        image_name = f"{self.samples:06d}.png"
        (self.directory / "images" / image_name).write_bytes(data)
        row = {k: v for k, v in frame.items() if k != "images"}
        row.update(sample_index=self.samples, pc_recorded_monotonic_ns=time.perf_counter_ns(),
                   image=f"images/{image_name}", image_sha256=hashlib.sha256(data).hexdigest())
        self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        self.samples += 1
        self.last_id = frame_id

    def finish(self, result, reason=None):
        if result not in {"success", "failure", "aborted"}:
            raise ValueError("未知episode结果")
        if not self.active:
            raise ValueError("没有正在录制的episode")
        try:
            self.stream.flush()
        finally:
            self.stream.close()
            self.stream = None
        summary = dict(result=result, samples=self.samples, complete=not self.incomplete and self.samples > 0,
                       reason=reason, is_real_demonstration=False, ended_at_utc=datetime.now(timezone.utc).isoformat())
        (self.directory / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        self.latest_episode = self.directory.name
        return self.latest_episode

    def state(self):
        return dict(active=self.active, samples=self.samples,
                    episode=self.directory.name if self.directory else None, latest_episode=self.latest_episode)

def read_raw_episode(path):
    """A process-interrupted recording remains inspectable and cannot be exported."""
    path = Path(path)
    rows = []
    truncated = False
    samples_file = path / "samples.jsonl"
    if samples_file.is_file():
        for line in samples_file.read_text(encoding="utf-8").splitlines():
            try: rows.append(json.loads(line))
            except (ValueError, TypeError): truncated = True; break
    summary_file = path / "summary.json"
    try:
        summary=json.loads(summary_file.read_text(encoding='utf-8'))
    except (FileNotFoundError,ValueError):
        summary=dict(result="aborted", samples=len(rows), complete=False, reason="录制结束摘要缺失或不完整", is_real_demonstration=False)
    if truncated: summary = dict(summary, complete=False, reason="原始记录末尾不完整")
    return summary, rows
