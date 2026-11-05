"""Opt-in camera inspection. Reads properties/calibration; starts requested streams.

No device-property setters, firmware, flash, reset, or calibration writes.
Quality statistics describe the observed scene, not a factory pass/fail test.
"""
from __future__ import annotations

import argparse
import ctypes as C
import csv
import json
import sys
import time
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

from obsdk import SDK, ROOT, P, I, U, S
from camera import Camera, FrameStats, decode_color, decode_depth


class Intrinsic(C.Structure):
    _pack_ = 1  # ObTypes.h explicitly uses pragma pack(push, 1).
    _fields_ = [(k, C.c_float) for k in ('fx', 'fy', 'cx', 'cy')] + [('width', C.c_int16), ('height', C.c_int16)]


class Distortion(C.Structure):
    _pack_ = 1
    _fields_ = [(k, C.c_float) for k in ('k1', 'k2', 'k3', 'k4', 'k5', 'k6', 'p1', 'p2')]


class Transform(C.Structure):
    _pack_ = 1
    _fields_ = [('rot', C.c_float * 9), ('trans', C.c_float * 3)]


class CameraParam(C.Structure):
    _pack_ = 1
    _fields_ = [('depthIntrinsic', Intrinsic), ('rgbIntrinsic', Intrinsic),
                ('depthDistortion', Distortion), ('rgbDistortion', Distortion),
                ('transform', Transform), ('isMirrored', C.c_bool)]


class PropertyItem(C.Structure):
    # Property.h is outside ObTypes.h's push/pop packing region.
    _fields_ = [('id', I), ('name', S), ('type', I), ('permission', I)]


assert C.sizeof(CameraParam) == 153


def bind(sdk):
    specs = {
        'device_get_sensor_list': (P, [P]), 'delete_sensor_list': (None, [P]),
        'sensor_list_get_sensor_count': (U, [P]), 'sensor_list_get_sensor_type': (I, [P, U]),
        'device_get_supported_property_count': (U, [P]),
        'device_get_supported_property': (PropertyItem, [P, U]),
        'device_get_float_property': (C.c_float, [P, I]),
        'pipeline_get_camera_param': (CameraParam, [P]),
        'pipeline_get_camera_param_with_profile': (CameraParam, [P, U, U, U, U]),
        'device_get_calibration_camera_param_list': (P, [P]),
        'camera_param_list_count': (U, [P]), 'camera_param_list_get_param': (CameraParam, [P, U]),
        'delete_camera_param_list': (None, [P]),
        'get_d2c_depth_profile_list': (P, [P, P, I]),
        'device_get_structured_data': (None, [P, I, P, C.POINTER(U)]),
        'device_is_global_timestamp_supported': (C.c_bool, [P]),
    }
    for name, (result, args) in specs.items():
        f = getattr(sdk.lib, 'ob_' + name)
        f.restype, f.argtypes = result, args + [C.POINTER(P)]
        sdk.funcs[name] = f


def plain(value):
    if isinstance(value, C.Structure):
        return {k: plain(getattr(value, k)) for k, _ in value._fields_}
    if isinstance(value, C.Array):
        return list(value)
    if isinstance(value, bytes):
        return value.decode('utf-8', 'replace')
    return value


def optional(fn):
    try:
        return fn()
    except RuntimeError as exc:
        return {'error': str(exc)}


def status(sdk, device):
    out = sdk.status(device)
    for name, prop, kind in [('ldp_distance_mm', 100, 'int'), ('color_auto_exposure', 2000, 'bool'),
                              ('color_exposure', 2001, 'int'), ('color_gain', 2002, 'int'), ('ir_gain', 2027, 'int'),
                              ('min_depth', 22, 'int'), ('max_depth', 23, 'int')]:
        out[name] = optional(lambda p=prop, k=kind: sdk.call(f'device_get_{k}_property', device, p)
                             if sdk.call('device_is_property_supported', device, p, 1) else 'unsupported')
    if 'device_get_structured_data' in sdk.funcs:
        out['temperature'] = optional(lambda: temperature(sdk, device))
    return out


def temperature(sdk, device):
    if not sdk.call('device_is_property_supported', device, 1003, 1):
        return 'unsupported'
    values = (C.c_float * 11)()
    size = U(C.sizeof(values))
    sdk.call('device_get_structured_data', device, 1003, C.byref(values), C.byref(size))
    names = ['cpu', 'ir', 'laser', 'mainboard', 'tec', 'imu', 'rgb', 'ir_left', 'ir_right', 'chip_top', 'chip_bottom']
    return {'returned_bytes': size.value, 'celsius_raw': dict(zip(names[:size.value//4], list(values)[:size.value//4])),
            'note': 'Raw SDK fields; sentinel/unimplemented fields are not physical temperatures.'}


def validate_calibration(param):
    checks = {}
    for name in ('depthIntrinsic', 'rgbIntrinsic'):
        k = param[name]
        checks[name] = bool(all(np.isfinite(k[x]) for x in ('fx', 'fy', 'cx', 'cy'))
                            and k['fx'] > 0 and k['fy'] > 0 and k['width'] > 0 and k['height'] > 0
                            and 0 <= k['cx'] < k['width'] and 0 <= k['cy'] < k['height'])
    r = np.asarray(param['transform']['rot']).reshape(3, 3)
    checks['rotation_determinant'] = float(np.linalg.det(r))
    checks['rotation_orthogonality_max_error'] = float(np.max(np.abs(r @ r.T - np.eye(3))))
    checks['translation_norm_mm'] = float(np.linalg.norm(param['transform']['trans']))
    checks['note'] = 'Numerical plausibility only; physical registration and accuracy require a target.'
    return checks


def inspect_device():
    sdk = SDK()
    bind(sdk)
    ctx = devices = device = pipeline = sensors = params = None
    report = {'sdk': '1.10.37', 'read_only': True}
    try:
        ctx = sdk.context()
        devices = sdk.call('query_device_list', ctx)
        if sdk.call('device_list_device_count', devices) != 1:
            raise RuntimeError('Inspection requires exactly one Orbbec camera.')
        device = sdk.call('device_list_get_device', devices, 0)
        report['device'] = sdk.info(device)
        report['global_timestamp_supported'] = optional(lambda: sdk.call('device_is_global_timestamp_supported', device))
        report['status'] = status(sdk, device)
        sensors = sdk.call('device_get_sensor_list', device)
        report['sensors'] = [sdk.call('sensor_list_get_sensor_type', sensors, i)
                             for i in range(sdk.call('sensor_list_get_sensor_count', sensors))]
        pipeline = sdk.call('create_pipeline_with_device', device)
        report['profiles'] = {str(s): optional(lambda s=s: sdk.profiles(pipeline, s)) for s in report['sensors'] if s in (1, 2, 3, 6, 7)}
        report['properties'] = optional(lambda: [plain(sdk.call('device_get_supported_property', device, i))
                                     for i in range(sdk.call('device_get_supported_property_count', device))])
        report['calibration_720p_480p'] = optional(lambda: plain(sdk.call('pipeline_get_camera_param_with_profile', pipeline, 1280, 720, 640, 480)))
        if 'error' not in report['calibration_720p_480p']:
            report['calibration_plausibility'] = validate_calibration(report['calibration_720p_480p'])
        report['calibration_640x360_640x480'] = optional(lambda: plain(sdk.call('pipeline_get_camera_param_with_profile', pipeline, 640, 360, 640, 480)))
        try:
            params = sdk.call('device_get_calibration_camera_param_list', device)
            report['stored_calibrations'] = [plain(sdk.call('camera_param_list_get_param', params, i)) for i in range(sdk.call('camera_param_list_count', params))]
        except RuntimeError as exc:
            report['stored_calibrations'] = {'error': str(exc)}
        color_list = color = None
        try:
            color_list = sdk.call('pipeline_get_stream_profile_list', pipeline, 2)
            color = sdk.call('stream_profile_list_get_video_stream_profile', color_list, 640, 360, 5, 30)
            report['d2c_color_profile'] = [640, 360, 5, 30]
            report['d2c_profiles'] = {}
            for mode in (1, 2):
                supported = None
                try:
                    supported = sdk.call('get_d2c_depth_profile_list', pipeline, color, mode)
                    entries = []
                    for n in range(sdk.call('stream_profile_list_count', supported)):
                        p = sdk.call('stream_profile_list_get_profile', supported, n)
                        try:
                            entries.append({**{k: sdk.call('video_stream_profile_' + k, p) for k in ('width', 'height', 'fps')},
                                            'format_id': sdk.call('stream_profile_format', p)})
                        finally:
                            sdk.release('stream_profile', p)
                    report['d2c_profiles'][str(mode)] = entries
                except RuntimeError as exc:
                    report['d2c_profiles'][str(mode)] = {'error': str(exc)}
                finally:
                    sdk.release('stream_profile_list', supported)
        finally:
            sdk.release('stream_profile', color)
            sdk.release('stream_profile_list', color_list)
    finally:
        for kind, handle in [('camera_param_list', params), ('sensor_list', sensors), ('pipeline', pipeline),
                             ('device', device), ('device_list', devices), ('context', ctx)]:
            sdk.release(kind, handle)
    return report


def depth_metrics(mm):
    valid = mm > 0
    h, w = mm.shape
    roi = mm[h//2-40:h//2+40, w//2-40:w//2+40]
    values = mm[valid]
    center = roi[roi > 0]
    return {'valid_fraction': float(valid.mean()), 'median_mm': float(np.median(values)) if values.size else None,
            'p05_mm': float(np.percentile(values, 5)) if values.size else None,
            'p95_mm': float(np.percentile(values, 95)) if values.size else None,
            'center_valid_fraction': float(np.mean(roi > 0)),
            'center_median_mm': float(np.median(center)) if center.size else None}


class PixelNoise:
    """Online per-pixel variation; zero-depth samples are excluded, never filled."""
    def __init__(self):
        self.n = self.mean = self.m2 = None

    def add(self, mm):
        h, w = mm.shape
        a = mm[h//2-40:h//2+40, w//2-40:w//2+40].astype(np.float64)
        if self.n is None:
            self.n, self.mean, self.m2 = np.zeros_like(a), np.zeros_like(a), np.zeros_like(a)
        mask = a > 0
        self.n[mask] += 1
        delta = a[mask] - self.mean[mask]
        self.mean[mask] += delta / self.n[mask]
        self.m2[mask] += delta * (a[mask] - self.mean[mask])

    def summary(self):
        mask = self.n >= 30 if self.n is not None else np.array([], dtype=bool)
        std = np.sqrt(self.m2[mask] / (self.n[mask]-1)) if mask.any() else np.array([])
        return {'pixels_with_30_samples': int(mask.sum()), 'median_pixel_std_mm': float(np.median(std)) if std.size else None,
                'p95_pixel_std_mm': float(np.percentile(std, 95)) if std.size else None,
                'note': 'Only interpretable as repeatability if camera and target stayed stationary; includes warm-up/drift.'}


PRESETS = {'both': {'color': (1280, 720, 5, 30), 'depth': (640, 480, 12, 30)},
           'color': {'color': (1280, 720, 5, 30)},
           '1080p': {'color': (1920, 1080, 5, 30), 'depth': (640, 480, 12, 30)},
           'depth': {'depth': (640, 480, 12, 30)}, 'ir': {'ir': (640, 480, 10, 30)},
           'd2c': {'color': (640, 360, 5, 30), 'depth': (640, 360, 12, 30)}}


def capture(run, preset, seconds, poll_status=False):
    camera = None
    report = {'preset': preset, 'requested_seconds': seconds, 'started_local': datetime.now().isoformat(), 'status': 'starting',
              'status_polling_during_capture': poll_status}
    stats, latest, last_seen, noise = {}, {}, {}, PixelNoise()
    valid_frames = zero_frames = longest_zero = consecutive_zero = 0
    saver = ThreadPoolExecutor(max_workers=1)
    pending_saves = []
    start = time.monotonic()
    try:
        camera = Camera('both', profiles=PRESETS[preset], align_mode=1 if preset == 'd2c' else 0)
        bind(camera.sdk)
        report.update(device=camera.info, profiles=camera.selected)
        if 'depth' in camera.selected and 'color' in camera.selected:
            report['calibration'] = optional(lambda: plain(camera.sdk.call('pipeline_get_camera_param', camera.pipeline)))
            if 'error' not in report['calibration']:
                report['calibration_plausibility'] = validate_calibration(report['calibration'])
        report['status_before'] = status(camera.sdk, camera.device)
        # Drain startup/control-query backlog before the timed performance interval.
        warmup_start, warmup_frames = time.monotonic(), 0
        while time.monotonic() - warmup_start < 1:
            warmup_frames += len(camera.read())
        report['warmup'] = {'seconds': time.monotonic()-warmup_start, 'frames_not_in_timed_statistics': warmup_frames}
        start = time.monotonic()
        stats = {k: FrameStats() for k in camera.selected}
        last_seen = {k: start for k in camera.selected}
        sample_due, save_due, progress_due = 1., 2., 30.
        with (run/'frames.csv').open('w', newline='', encoding='utf-8') as f, (run/'samples.jsonl').open('w', encoding='utf-8') as samples:
            writer = None
            while time.monotonic() - start < seconds:
                for meta, data in camera.read():
                    name = meta['stream']
                    stats[name].add(meta)
                    last_seen[name] = meta['host_monotonic_ns'] / 1e9
                    if writer is None:
                        writer = csv.DictWriter(f, fieldnames=list(meta))
                        writer.writeheader()
                    writer.writerow(meta)
                    w, h, fmt = meta['width'], meta['height'], meta['format_id']
                    if name == 'color':
                        a = decode_color(data, w, h, fmt)
                    elif name == 'depth':
                        _, a = decode_depth(data, w, h, fmt, meta['depth_scale_mm'])
                        any_valid = bool(np.any(a > 0))
                        valid_frames += any_valid
                        zero_frames += not any_valid
                        consecutive_zero = 0 if any_valid else consecutive_zero + 1
                        longest_zero = max(longest_zero, consecutive_zero)
                        noise.add(a)
                    else:
                        if fmt not in (8, 9, 10, 11, 12):
                            raise ValueError(f'Unsupported IR format {fmt}')
                        a = np.frombuffer(data, dtype=np.uint8 if fmt == 9 else '<u2').reshape(h, w).copy()
                    latest[name] = a
                now = time.monotonic() - start
                if any(time.monotonic()-v > 3 for v in last_seen.values()):
                    raise RuntimeError('Selected stream missing for >3 seconds; no automatic reconnect.')
                if now >= sample_due:
                    sample = {'elapsed_s': now}
                    if poll_status:
                        sample['status'] = status(camera.sdk, camera.device)
                    if 'depth' in latest:
                        sample['depth'] = depth_metrics(latest['depth'])
                    if 'color' in latest:
                        gray = cv2.cvtColor(latest['color'], cv2.COLOR_BGR2GRAY)
                        sample['color'] = {'mean_luma': float(gray.mean()), 'black_fraction': float(np.mean(gray <= 2)),
                                           'white_fraction': float(np.mean(gray >= 253)), 'laplacian_variance': float(cv2.Laplacian(gray, cv2.CV_32F).var())}
                    if 'ir' in latest:
                        sample['ir'] = {'min': int(latest['ir'].min()), 'max': int(latest['ir'].max()), 'mean': float(latest['ir'].mean()),
                                        'std': float(latest['ir'].std()), 'zero_fraction': float(np.mean(latest['ir'] == 0))}
                    samples.write(json.dumps(sample)+'\n')
                    samples.flush()
                    report['last_sample'] = sample
                    sample_due += 1
                if now >= save_due:
                    # PNG encoding/disk I/O must not block the SDK receive loop.
                    remaining = []
                    for future in pending_saves:
                        if future.done():
                            future.result()
                        else:
                            remaining.append(future)
                    pending_saves = remaining
                    if len(pending_saves) >= 2:
                        raise RuntimeError('Snapshot writer cannot keep up; refusing unbounded buffering.')
                    pending_saves.append(saver.submit(save_snapshot, run, int(now), dict(latest)))
                    save_due += 60
                if now >= progress_due:
                    print(json.dumps({'elapsed_s': round(now, 1), 'frames': {k: s.count for k,s in stats.items()},
                                      'zero_depth_frames': zero_frames, 'last_sample': report.get('last_sample') }), flush=True)
                    progress_due += 30
            report['elapsed_s'] = time.monotonic()-start
            report['status_after'] = status(camera.sdk, camera.device)
            pending_saves.append(saver.submit(save_snapshot, run, int(seconds), dict(latest)))
            for future in pending_saves:
                future.result()
        report['status'] = 'completed'
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
    finally:
        saver.shutdown(wait=True)
        report.setdefault('elapsed_s', time.monotonic()-start)
        report['cleanup_errors'] = camera.close() if camera else []
        if report['cleanup_errors']:
            report['status'] = 'failed'
        report['streams'] = {k: s.summary() for k, s in stats.items()}
        report['depth_quality'] = {'valid_frames': valid_frames, 'zero_frames': zero_frames, 'longest_zero_run_frames': longest_zero,
                                    'center_temporal_variation': noise.summary()}
    return report


def save_snapshot(run, second, latest):
    for name, a in latest.items():
        if name == 'color':
            display = a
        elif name == 'depth':
            np.save(run/f'{second:04d}_depth_mm.npy', a)
            display = cv2.applyColorMap(np.clip((a-200)/2300*255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            display[a == 0] = 0
        else:
            np.save(run/f'{second:04d}_ir.npy', a)
            lo, hi = np.percentile(a, [1, 99])
            display = np.clip((a.astype(np.float32)-lo) / max(hi-lo, 1) * 255, 0, 255).astype(np.uint8)
        if not cv2.imwrite(str(run/f'{second:04d}_{name}.png'), display):
            raise RuntimeError('Snapshot write failed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--inspect', action='store_true')
    parser.add_argument('--preset', choices=PRESETS, default='both')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--poll-status', action='store_true', help='Poll every second; control transfers can delay frames. Not for performance acceptance.')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 1800:
        parser.error('seconds must be 1..1800')
    run = ROOT/'captures'/('health_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir(parents=True)
    print(f'Results: {run}', flush=True)
    report = inspect_device() if args.inspect else capture(run, args.preset, args.seconds, args.poll_status)
    (run/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    # Full enumerations stay in the file, not the console.
    print(json.dumps({k: v for k,v in report.items() if k not in ('profiles','properties','stored_calibrations','d2c_profiles','calibration',
                        'calibration_720p_480p','calibration_640x360_640x480','status_before','status_after')}, ensure_ascii=False, indent=2), flush=True)
    return int(report.get('status') in ('failed','interrupted'))


if __name__ == '__main__':
    sys.exit(main())
