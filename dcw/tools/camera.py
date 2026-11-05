"""Enumerated RGB/depth acquisition and local preview through official SDK v1.

No D2C, hardware synchronization, exposure changes, firmware or flash writes.
Both streams retain their own frame metadata. RGB and depth are not aligned.
"""
from __future__ import annotations

import argparse
import csv
import ctypes as C
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from obsdk import SDK, ROOT, FORMATS


class FrameStats:
    """Constant-memory stream statistics; full metadata goes to frames.csv."""
    def __init__(self):
        self.count = self.zero_timestamps = self.index_nonmonotonic = self.index_gaps = self.timestamp_nonmonotonic = 0
        self.first_ns = self.last_ns = self.last_index = self.last_stamp = self.first_stamp = None
        self.max_gap_ns = 0

    def add(self, meta):
        ns = meta.get("host_perf_counter_ns", meta["host_monotonic_ns"])
        index, stamp = meta["index"], meta["device_timestamp_us"]
        if self.last_ns is not None:
            self.max_gap_ns = max(self.max_gap_ns, ns - self.last_ns)
            self.index_nonmonotonic += index <= self.last_index
            self.index_gaps += max(0, index - self.last_index - 1)
            self.timestamp_nonmonotonic += stamp <= self.last_stamp
        else:
            self.first_ns = ns
            self.first_stamp = stamp
        self.last_ns, self.last_index, self.last_stamp = ns, index, stamp
        self.count += 1
        self.zero_timestamps += stamp == 0

    def summary(self):
        elapsed = (self.last_ns - self.first_ns) / 1e9 if self.count else 0
        stamp_ratio = (self.last_stamp - self.first_stamp) / (elapsed * 1e6) if elapsed > 0 else None
        return {"frames": self.count, "host_fps_between_first_last": (self.count-1)/elapsed if elapsed > 0 else None,
                "max_host_gap_ms": self.max_gap_ns / 1e6, "index_nonmonotonic_count": self.index_nonmonotonic,
                "index_gaps": self.index_gaps, "device_timestamp_nonmonotonic_count": self.timestamp_nonmonotonic,
                "device_timestamp_zero_count": self.zero_timestamps,
                "device_timestamp_elapsed_over_host_elapsed": stamp_ratio,
                "device_timestamp_rate_plausible_as_us": .95 < stamp_ratio < 1.05 if stamp_ratio is not None else None}


def decode_color(data, width, height, fmt):
    raw = np.frombuffer(data, dtype=np.uint8)
    if fmt == 5:
        bgr = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if bgr is None or bgr.shape[:2] != (height, width):
            raise ValueError("Invalid MJPEG frame")
        return bgr
    if fmt in (22, 23):
        frame = raw.reshape(height, width, 3)
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if fmt == 22 else frame.copy()
    if fmt == 25:
        return cv2.cvtColor(raw.reshape(height, width, 4), cv2.COLOR_BGRA2BGR)
    if fmt in (0, 1, 2):
        return cv2.cvtColor(raw.reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY if fmt == 2 else cv2.COLOR_YUV2BGR_YUY2)
    if fmt == 3:
        return cv2.cvtColor(raw.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_NV12)
    raise ValueError(f"Unsupported color format {fmt}")


def decode_depth(data, width, height, fmt, scale):
    if fmt not in (8, 11, 12, 28) or len(data) != width * height * 2:
        raise ValueError(f"Expected unpacked 16-bit depth, got format={fmt}, bytes={len(data)}")
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError(f"Invalid depth scale {scale}")
    raw = np.frombuffer(data, dtype="<u2").reshape(height, width).copy()
    return raw, raw.astype(np.float32) * scale


class Camera:
    def __init__(self, mode, serial=None, *, profiles=None, align_mode=0, uvc_pts=False):
        self.sdk = SDK()
        self.ctx = self.devices = self.device = self.pipeline = self.config = None
        self.started = False
        self.selected = {}
        self.uvc_clock = self.uvc_clock_info = None
        try:
            self.ctx = self.sdk.context()
            self.devices = self.sdk.call("query_device_list", self.ctx)
            count = self.sdk.call("device_list_device_count", self.devices)
            if count == 0:
                raise RuntimeError("No RGB-D device. Check USB and Orbbec depth driver; try replugging.")
            if count > 1 and not serial:
                raise RuntimeError("Multiple devices found. Select one using --serial.")
            for index in range(count):
                candidate = self.sdk.call("device_list_get_device", self.devices, index)
                try:
                    info = self.sdk.info(candidate)
                    if not serial or info["serial_number"] == serial:
                        self.device, self.info = candidate, info
                        candidate = None
                        break
                finally:
                    self.sdk.release("device", candidate)
            if not self.device:
                raise RuntimeError(f"Serial {serial!r} not found")
            if uvc_pts:
                from uvc_clock import gemini_rgb_clock, PtsClock
                self.uvc_clock_info = gemini_rgb_clock(self.info)
                self.uvc_clock = PtsClock(self.uvc_clock_info['clock_hz'])
            self.pipeline = self.sdk.call("create_pipeline_with_device", self.device)
            self.config = self.sdk.call("create_config")
            self.sdk.call("config_set_align_mode", self.config, align_mode)
            self.sdk.call("config_set_frame_aggregate_output_mode", self.config, 3)
            requested = profiles if profiles is not None else {
                name: wanted for name, wanted in (("color", (1280, 720, 5, 30)), ("depth", (640, 480, 12, 30)))
                if mode == "both" or name == mode}
            for name, wanted in requested.items():
                sensor = {"ir": 1, "color": 2, "depth": 3}[name]
                profiles = self.sdk.call("pipeline_get_stream_profile_list", self.pipeline, sensor)
                profile = None
                try:
                    # Exact requested profile only. Never silently substitute resolution/fps.
                    profile = self.sdk.call("stream_profile_list_get_video_stream_profile", profiles, *wanted)
                    self.sdk.call("config_enable_stream", self.config, profile)
                    w, h, fmt, fps = wanted
                    self.selected[name] = {"width": w, "height": h, "format_id": fmt, "format": FORMATS[fmt], "fps": fps}
                finally:
                    self.sdk.release("stream_profile", profile)
                    self.sdk.release("stream_profile_list", profiles)
            self.sdk.call("pipeline_start_with_config", self.pipeline, self.config)
            self.started = True
        except BaseException:
            self.close()
            raise

    def read(self):
        frameset = self.sdk.call("pipeline_wait_for_frameset", self.pipeline, 200)
        received_ns = time.monotonic_ns()
        received_perf_ns = time.perf_counter_ns()
        if not frameset:
            return []
        result = []
        try:
            for name in self.selected:
                frame = self.sdk.call(f"frameset_{name}_frame", frameset)
                if not frame:
                    continue
                try:
                    metadata = {"stream": name, "host_monotonic_ns": received_ns, "host_perf_counter_ns": received_perf_ns}
                    for key, func in (("index", "frame_index"), ("device_timestamp_us", "frame_time_stamp_us"),
                                      ("sdk_system_timestamp_us", "frame_system_time_stamp_us"), ("format_id", "frame_format"),
                                      ("width", "video_frame_width"), ("height", "video_frame_height"), ("bytes", "frame_data_size")):
                        metadata[key] = self.sdk.call(func, frame)
                    metadata["depth_scale_mm"] = self.sdk.call("depth_frame_get_value_scale", frame) if name == "depth" else None
                    if self.uvc_clock:
                        keys = ('uvc_pts_32', 'uvc_pts_unwrapped', 'uvc_pts_clock_hz', 'uvc_pts_ns', 'uvc_pts_wrap_count', 'uvc_kernel_receive_ns', 'uvc_pts_available')
                        metadata.update({k: None for k in keys})
                        if name == 'color':
                            from uvc_clock import v4l2_pts
                            size = self.sdk.call('frame_metadata_size', frame)
                            if size > 65536:
                                raise ValueError('Unexpected UVC metadata size')
                            metadata['uvc_pts_available'] = bool(size)
                            # V4L2 can omit headers, especially at startup. Keep
                            # the image and an explicitly unavailable PTS field.
                            if size:
                                raw = C.string_at(self.sdk.call('frame_metadata', frame), size)
                                pts, kernel_ns = v4l2_pts(raw, received_ns)
                                metadata.update(self.uvc_clock.add(pts), uvc_kernel_receive_ns=kernel_ns)
                    data = C.string_at(self.sdk.call("frame_data", frame), metadata["bytes"])
                    result.append((metadata, data))
                finally:
                    self.sdk.release("frame", frame)
        finally:
            self.sdk.release("frame", frameset)
        return result

    def close(self):
        errors = []
        if self.started:
            try:
                self.sdk.call("pipeline_stop", self.pipeline)
            except Exception as exc:
                errors.append(str(exc))
            self.started = False
        for kind in ("config", "pipeline", "device", "device_list", "context"):
            attr = {"device_list": "devices", "context": "ctx"}.get(kind, kind)
            try:
                self.sdk.release(kind, getattr(self, attr))
            except Exception as exc:
                errors.append(str(exc))
            setattr(self, attr, None)
        return errors


def render_preview(latest, counts, started, info, depth_valid=None):
    canvas = np.full((580, 1280, 3), (24, 21, 18), dtype=np.uint8)
    cv2.putText(canvas, f"{info['name']} | {info['serial_number']} | RGB + DEPTH (not aligned)", (20, 30), 0, 0.7, (240, 240, 240), 1, cv2.LINE_AA)
    now = time.monotonic()
    for name, x in (("color", 0), ("depth", 640)):
        if name in latest:
            im, received = latest[name]
            size = (640, 360) if name == "color" else (640, 480)
            thumb = cv2.resize(im, size, interpolation=cv2.INTER_NEAREST if name == "depth" else cv2.INTER_AREA)
            y = 75 + (480 - size[1]) // 2
            canvas[y:y+size[1], x:x+640] = thumb
            stale = now - received > 0.5
            label = f"{name.upper()} {counts[name] / max(now-started, .01):.1f} fps"
            if name == "depth":
                label += f" | valid {depth_valid*100:.0f}%" if depth_valid is not None else ""
                if depth_valid == 0:
                    label += "  NO VALID DEPTH"
            if stale:
                canvas[y:y+size[1], x:x+640] //= 3
                label += "  STALE"
            cv2.putText(canvas, label, (x+15, 65), 0, 0.6, (80, 100, 255) if stale else (210, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Q / ESC: stop | Depth color scale: 0.2-2.5m; black = invalid | Original image orientation", (20, 575), 0, 0.45, (175, 180, 180), 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("color", "depth", "both"), default="both")
    parser.add_argument("--serial")
    parser.add_argument("--seconds", type=float, default=30, help="0 = until preview window closes")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--snapshots", action="store_true")
    args = parser.parse_args()
    if args.seconds < 0 or (args.seconds == 0 and not args.preview):
        parser.error("Use positive seconds, or --preview --seconds 0")
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run = ROOT / "captures" / run_id
    run.mkdir(parents=True)
    report = {"run_id": run_id, "mode": args.mode, "requested_seconds": args.seconds,
              "sdk": "1.10.37", "alignment": "disabled", "hardware_sync": False,
              "timestamp_note": "SDK-reported timestamps, not independently validated exposure times", "status": "starting"}
    camera = None
    counts, events, latest = {}, {}, {}
    error = None
    start = time.monotonic()
    try:
        camera = Camera(args.mode, args.serial)
        report.update(device=camera.info, selected_profiles=camera.selected, read_only_status=camera.sdk.status(camera.device))
        print(json.dumps(report, ensure_ascii=False), flush=True)
        counts = {name: 0 for name in camera.selected}
        events = {name: FrameStats() for name in camera.selected}
        depth_valid_frames = 0
        last_received = {name: time.monotonic() for name in camera.selected}
        start = time.monotonic()
        if args.preview:
            cv2.namedWindow("Orbbec RGB-D Preview", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Orbbec RGB-D Preview", 1280, 580)
        with (run / "frames.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["stream", "host_monotonic_ns", "host_perf_counter_ns", "index", "device_timestamp_us", "sdk_system_timestamp_us", "format_id", "width", "height", "bytes", "depth_scale_mm"])
            writer.writeheader()
            saved = set()
            while args.seconds == 0 or time.monotonic() - start < args.seconds:
                for meta, data in camera.read():
                    name = meta["stream"]
                    counts[name] += 1
                    events[name].add(meta)
                    writer.writerow(meta)
                    last_received[name] = meta["host_monotonic_ns"] / 1e9
                    w, h, fmt = meta["width"], meta["height"], meta["format_id"]
                    if name == "color":
                        display = decode_color(data, w, h, fmt)
                    else:
                        raw, mm = decode_depth(data, w, h, fmt, meta["depth_scale_mm"])
                        colored = np.clip((mm - 200) / 2300 * 255, 0, 255).astype(np.uint8)
                        display = cv2.applyColorMap(colored, cv2.COLORMAP_TURBO)
                        display[raw == 0] = 0
                        valid = mm[raw > 0]
                        depth_valid_frames += bool(valid.size)
                        report["last_depth"] = {"scale_mm": meta["depth_scale_mm"], "valid_fraction": float(np.mean(raw > 0)),
                                                "median_mm": float(np.median(valid)) if valid.size else None}
                    latest[name] = (display, last_received[name])
                    if args.snapshots and name not in saved and time.monotonic()-start > 2:
                        if not cv2.imwrite(str(run / f"{name}.png"), display):
                            raise RuntimeError("Snapshot write failed")
                        if name == "depth":
                            np.save(run / "depth_raw.npy", raw)
                            (run / "depth_snapshot.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
                        saved.add(name)
                if any(time.monotonic() - t > 3 for t in last_received.values()):
                    raise RuntimeError("A selected stream has no fresh frame for 3 seconds. Stopped; no automatic reconnect.")
                if args.preview:
                    cv2.imshow("Orbbec RGB-D Preview", render_preview(latest, counts, start, camera.info, report.get("last_depth", {}).get("valid_fraction")))
                    if cv2.waitKey(1) & 0xff in (27, ord("q")) or cv2.getWindowProperty("Orbbec RGB-D Preview", cv2.WND_PROP_VISIBLE) < 1:
                        break
        report["status"] = "completed"
        if "depth" in camera.selected:
            report["depth_frames_with_valid_pixels"] = depth_valid_frames
            report["data_validation"] = "valid_depth_observed" if depth_valid_frames else "no_valid_depth_observed"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
    except Exception as exc:
        error = str(exc)
        report.update(status="failed", error=error)
    finally:
        report["elapsed_s"] = time.monotonic() - start
        report["cleanup_errors"] = camera.close() if camera else []
        if report["cleanup_errors"]:
            report["status"] = "failed"
        if args.preview:
            cv2.destroyAllWindows()
        report["streams"] = {}
        for name, stats in events.items():
            report["streams"][name] = stats.summary()
        (run / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print(f"Results: {run}", flush=True)
    return 1 if report["status"] == "failed" else 0


if __name__ == "__main__":
    sys.exit(main())
