"""Local Dataset v3 conversion through the pinned official LeRobot writer/reader."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy as np
from PIL import Image

LEROBOT_COMMIT = "8c920c4270460851cedd2737657584586d3dc66f"
IMAGE_KEY = "observation.images.wrist"
NAMES = ["joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad", "joint_5_rad", "joint_6_rad", "gripper_open_fraction"]


def load_episode(path):
    path = Path(path).resolve()
    metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    if metadata.get("dataset_kind") != "simulation" or summary.get("result") not in {"success", "failure"} or not summary.get("complete"):
        raise ValueError("Only complete success/failure simulation episodes may be exported")
    rows = [json.loads(line) for line in (path / "samples.jsonl").read_text(encoding="utf-8").splitlines()]
    if not rows or len(rows) != summary["samples"]:
        raise ValueError("Raw episode sample count mismatch")
    previous = None
    for index, row in enumerate(rows):
        o, a = row["observation"], row["effective_action"]
        if row["sample_index"] != index or not o["valid"] or not o["is_simulated"]:
            raise ValueError("Invalid or non-simulated observation")
        if previous and (o["frame_id"] != previous["frame_id"] + 1 or o["gateway_monotonic_ns"] <= previous["gateway_monotonic_ns"]):
            raise ValueError("Raw episode frame gap or time reversal")
        if a["gateway_applied_ns"] < o["gateway_monotonic_ns"]:
            raise ValueError("Action precedes observation")
        for value in (state_vector(row), action_vector(row)):
            if value.shape != (7,) or not np.isfinite(value).all() or not 0 <= value[6] <= 1:
                raise ValueError("Invalid 7D state/action")
        image = (path / row["image"]).resolve()
        if not image.is_relative_to(path / "images") or hashlib.sha256(image.read_bytes()).hexdigest() != row["image_sha256"]:
            raise ValueError("Image path or hash mismatch")
        with Image.open(image) as frame:
            if frame.size != (640, 360) or frame.mode != "RGB":
                raise ValueError("Expected RGB 640x360 image")
        previous = o
    return path, metadata, summary, rows


def state_vector(row):
    o = row["observation"]
    return np.asarray([*o["joints_rad"], o["gripper"]], dtype=np.float32)


def action_vector(row):
    a = row["effective_action"]
    return np.asarray([*a["joints_rad"], a["gripper"]], dtype=np.float32)


def validate_dataset(root, sources):
    import av
    import torch
    from torch.utils.data import DataLoader
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("local/e6_simulation", root=root, video_backend="pyav")
    total = sum(len(source[3]) for source in sources)
    assert len(ds) == total and ds.num_episodes == len(sources)
    offset, checks = 0, []
    for episode_index, (path, metadata, summary, rows) in enumerate(sources):
        # Official table reader checks every label, separately from video decoding.
        table = ds.hf_dataset.select(range(offset, offset + len(rows)))
        np.testing.assert_array_equal(np.asarray(table["action"]), np.stack([action_vector(r) for r in rows]))
        np.testing.assert_array_equal(np.asarray(table["observation.state"]), np.stack([state_vector(r) for r in rows]))
        assert set(int(v) for v in table["episode_index"]) == {episode_index}
        assert list(int(v) for v in table["frame_index"]) == list(range(len(rows)))
        np.testing.assert_allclose(table["timestamp"], np.arange(len(rows)) / 20, atol=1e-4)
        sample_indices = sorted({0, len(rows)//2, len(rows)-1})
        errors = []
        for i in sample_indices:
            item = ds[offset + i]
            assert item[IMAGE_KEY].shape == (3, 360, 640)
            pixels = item[IMAGE_KEY].permute(1, 2, 0).numpy() * 255
            with Image.open(path / rows[i]["image"]) as image:
                errors.append(float(np.abs(pixels - np.asarray(image, dtype=np.float32)).mean()))
        assert max(errors) < 12, f"Video/image association check failed: {errors}"
        checks.append(dict(raw_episode=path.name, result=summary["result"], frames=len(rows),
                           dataset_from=offset, dataset_to=offset+len(rows), video_image_mae=errors))
        offset += len(rows)
    videos = []
    for video in sorted(Path(root).glob("videos/**/*.mp4")):
        with av.open(str(video)) as container:
            frames = 0
            previous_pts = None
            for frame in container.decode(video=0):
                assert frame.pts is not None and (previous_pts is None or frame.pts > previous_pts)
                previous_pts = frame.pts
                assert (frame.width, frame.height) == (640, 360)
                frames += 1
            videos.append(dict(path=str(video.relative_to(root)), frames=frames, codec=container.streams.video[0].codec_context.name))
    assert sum(v["frames"] for v in videos) == total
    batch = next(iter(DataLoader(ds, batch_size=min(8, total), num_workers=0, shuffle=False)))
    assert batch["action"].shape == (min(8, total), 7)
    assert batch["observation.state"].shape == (min(8, total), 7)
    window = LeRobotDataset("local/e6_simulation", root=root, video_backend="pyav",
                            delta_timestamps={"action": [-.05, 0, .05], IMAGE_KEY: [-.05, 0, .05]})
    for check in checks:
        for index, expected_padding in ((check["dataset_from"], [True, False, False]), (check["dataset_to"]-1, [False, False, True])):
            item = window[index]
            assert item["action"].shape == (3, 7) and item[IMAGE_KEY].shape == (3, 3, 360, 640)
            if check["frames"]>1:
                assert item["action_is_pad"].tolist() == expected_padding
                assert item[IMAGE_KEY+"_is_pad"].tolist() == expected_padding
    return dict(validated=True, dataset_version=ds.meta.info.codebase_version, frames=total,
                episodes=checks, videos=videos, dataloader_action_shape=list(batch["action"].shape),
                dataloader_image_shape=list(batch[IMAGE_KEY].shape), temporal_window_boundary_padding=True,
                temporal_window_internal_episode_padding=True,
                video_backend="pyav", video_encoder="libx264 CPU", torch_version=torch.__version__,
                simulated=True, uploaded=False, action_labels="effective_action of same raw sample, no next-state substitution")


def export_episode(episodes, output):
    from lerobot.configs import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import importlib.metadata
    distribution = importlib.metadata.distribution("lerobot")
    direct = json.loads(distribution.read_text("direct_url.json"))
    if direct.get("vcs_info", {}).get("commit_id") != LEROBOT_COMMIT:
        raise RuntimeError("LeRobot does not match pinned commit; use uv sync --frozen --extra dataset")
    if isinstance(episodes, (str, Path)): episodes = [episodes]
    sources = [load_episode(path) for path in episodes]
    output = Path(output).resolve()
    if output.exists(): raise FileExistsError(f"Output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = output.with_name(output.name + ".partial")
    if stage.exists(): raise FileExistsError(f"Previous unfinished export: {stage}")
    features = {"observation.state": dict(dtype="float32", shape=(7,), names=NAMES),
                "action": dict(dtype="float32", shape=(7,), names=NAMES),
                "next.success": dict(dtype="bool", shape=(1,), names=None),
                IMAGE_KEY: dict(dtype="video", shape=(360, 640, 3), names=["height", "width", "channels"])}
    writer = LeRobotDataset.create("local/e6_simulation", fps=20, root=stage, features=features,
                                   robot_type="e6_kinematic_simulation", video_backend="pyav",
                                   rgb_encoder=RGBEncoderConfig(vcodec="h264", crf=18, preset="fast", g=2),
                                   encoder_threads=2, image_writer_threads=2)
    try:
        for path, metadata, summary, rows in sources:
            for row in rows:
                with Image.open(path / row["image"]) as image:
                    writer.add_frame({"observation.state": state_vector(row), "action": action_vector(row),
                                      "next.success": np.asarray([summary["result"] == "success"], dtype=bool),
                                      IMAGE_KEY: np.asarray(image).copy(), "task": metadata["task"]})
            writer.save_episode(parallel_encoding=False)
    finally:
        writer.finalize()
    report = validate_dataset(stage, sources)
    report.update(lerobot_commit=LEROBOT_COMMIT, timestamp_semantics="index/20 in LeRobot; original process monotonic clocks in raw records",
                  raw_sources=[str(s[0]) for s in sources])
    (stage / "simulation_validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    shutil.move(str(stage), str(output))
    print(json.dumps(dict(output=str(output), **report), ensure_ascii=False, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("episodes", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try: export_episode(args.episodes, args.output)
    except Exception as exc:
        print(f"Export failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise


if __name__ == "__main__": main()
