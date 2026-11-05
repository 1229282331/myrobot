"""Bounded, opt-in RGB-D/IR capture with an independent receiving thread.

No device-property setters, time sync, firmware, calibration or flash writes.
Preserves original SDK timestamps. Queue overflow and missing streams abort;
there is no automatic reconnect or silent application-frame replacement.
"""
from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor, wait
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import queue
import signal
import threading
import time

import cv2
import numpy as np

from camera import Camera, FrameStats, decode_color, decode_depth
from healthcheck import (PRESETS, bind, depth_metrics, optional, plain,
                         save_snapshot, status, validate_calibration)
from obsdk import SDK_ROOT, SDK_CONFIG

PROFILES = {**PRESETS, 'ir-native': {'ir': (512, 384, 10, 30)},
            'ir-high': {'ir': (1024, 768, 10, 10)}}


def decode_item(meta, data, decode_rgb=True):
    """Decode copied frame bytes; workers never access SDK handles or metadata."""
    started = time.perf_counter_ns()
    name, w, h, fmt = meta['stream'], meta['width'], meta['height'], meta['format_id']
    if name == 'color':
        im = decode_color(data, w, h, fmt) if decode_rgb else None
    elif name == 'depth':
        _, im = decode_depth(data, w, h, fmt, meta['depth_scale_mm'])
    else:
        if fmt not in (8, 9, 10, 11, 12):
            raise ValueError(f'Unsupported IR format {fmt}')
        im = np.frombuffer(data, dtype=np.uint8 if fmt == 9 else '<u2').reshape(h, w).copy()
    finished = time.perf_counter_ns()
    return im, started, finished


def ordered_frames(inbox, receiver_done, pool, workers, report, color_decode_stride, decoder=decode_item):
    """Keep at most 2*workers futures and emit in reception order.

    Continue feeding workers while the oldest result runs, so two JPEG frames
    can decode concurrently. A failed decode propagates instead of dropping it.
    """
    pending = deque()
    submitted_color = 0
    while not receiver_done.is_set() or not inbox.empty() or pending:
        while len(pending) < 2 * workers:
            try:
                meta, data = inbox.get(timeout=.05) if not pending else inbox.get_nowait()
            except queue.Empty:
                break
            decode_rgb = meta['stream'] != 'color' or submitted_color % color_decode_stride == 0
            if meta['stream'] == 'color':
                submitted_color += 1
            pending.append((meta, pool.submit(decoder, meta, data, decode_rgb)))
            report['decode_pending_max'] = max(report['decode_pending_max'], len(pending))
        if not pending:
            continue
        if not pending[0][1].done():
            wait([pending[0][1]], timeout=.002)
            continue
        meta, future = pending.popleft()
        im, started, finished = future.result()
        yield meta, im, started, finished


def capture(output, preset, seconds, stop_file=None, color_decode_stride=1, decode_workers=1, uvc_pts=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP) if hasattr(signal, 'SIGHUP') else (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    cv2.setNumThreads(1)
    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'status': 'starting',
              'preset': preset, 'requested_seconds': seconds, 'sdk': '1.10.37',
              'opencv_threads': cv2.getNumThreads(), 'opencv_version': cv2.__version__,
              'numpy_version': np.__version__, 'sdk_root': str(SDK_ROOT),
              'sdk_config': str(SDK_CONFIG), 'decode_workers': decode_workers,
              'uvc_pts_requested': uvc_pts,
              'uvc_pts_availability': {'valid_color_frames': 0, 'missing_color_frames': 0},
              'uvc_pts_semantics': 'Descriptor-frequency device PTS; exposure reference and host/RGB-D/robot synchronization unvalidated',
              'decode_pending_capacity': 2 * decode_workers, 'decode_pending_max': 0,
              'snapshot_saving': 'one asynchronous worker, at most one outstanding snapshot',
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'alignment_mode': 1 if preset == 'd2c' else 0,
              'clock_semantics': 'SDK fields unchanged; host receipt is not exposure time or synchronized observation',
              'receiver_queue_capacity': 128, 'receiver_queue_max_size': 0,
              'receiver_queue_overflow': False, 'receiver_errors': [], 'processing_errors': [],
              'color_decode_stride': color_decode_stride, 'color_decoded_frames': 0,
              'color_validation': 'Every color frame decoded' if color_decode_stride == 1 else 'Sampled decoding only; all received frame metadata retained',
              'warmup_s': 2, 'depth_quality': {'valid_frames': 0, 'zero_frames': 0}}
    camera = receiver = None
    latest, stats, seen, processed = {}, {}, {}, {}
    inbox = queue.Queue(maxsize=128)
    receiver_done = threading.Event()
    decode_pool = ThreadPoolExecutor(max_workers=decode_workers, thread_name_prefix='frame-decode')
    saver = ThreadPoolExecutor(max_workers=1, thread_name_prefix='snapshot-save')
    pending_save = None
    start = None
    try:
        camera = Camera('both', profiles=PROFILES[preset], align_mode=report['alignment_mode'], uvc_pts=uvc_pts)
        report['uvc_clock'] = camera.uvc_clock_info
        bind(camera.sdk)
        report.update(device=camera.info, profiles=camera.selected,
                      status_before=status(camera.sdk, camera.device))
        if 'color' in camera.selected and 'depth' in camera.selected:
            report['calibration'] = optional(lambda: plain(camera.sdk.call('pipeline_get_camera_param', camera.pipeline)))
            if 'error' not in report['calibration']:
                report['calibration_plausibility'] = validate_calibration(report['calibration'])
        # Warm up the codecs and image/statistical operations, as well as SDK reads.
        warmup_start = time.monotonic()
        warmup_frames = 0
        while time.monotonic() - warmup_start < 2:
            for meta, data in camera.read():
                name = meta['stream']
                if name == 'color':
                    im = decode_color(data, meta['width'], meta['height'], meta['format_id'])
                    cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
                elif name == 'depth':
                    _, im = decode_depth(data, meta['width'], meta['height'], meta['format_id'], meta['depth_scale_mm'])
                    depth_metrics(im)
                warmup_frames += 1
            if stop.is_set():
                raise RuntimeError('Cancelled during warmup')
        report['warmup_frames_excluded'] = warmup_frames
        stats = {k: FrameStats() for k in camera.selected}
        processed = {k: 0 for k in camera.selected}
        start = time.monotonic()
        seen = {k: start for k in camera.selected}
        report['timed_start_monotonic_ns'] = time.monotonic_ns()

        def receive():
            try:
                with (output / 'frames.csv').open('x', newline='', encoding='utf-8') as f:
                    writer = None
                    while not stop.is_set() and time.monotonic() - start < seconds:
                        if stop_file and Path(stop_file).exists():
                            stop.set()
                            raise RuntimeError('Stop file requested')
                        frames = camera.read()
                        for meta, data in frames:
                            name = meta['stream']
                            if uvc_pts and name == 'color':
                                key = 'valid_color_frames' if meta['uvc_pts_available'] else 'missing_color_frames'
                                report['uvc_pts_availability'][key] += 1
                            stats[name].add(meta)
                            seen[name] = meta['host_monotonic_ns'] / 1e9
                            if writer is None:
                                writer = csv.DictWriter(f, fieldnames=list(meta))
                                writer.writeheader()
                            writer.writerow(meta)
                            try:
                                inbox.put_nowait((meta, data))
                            except queue.Full:
                                report['receiver_queue_overflow'] = True
                                raise RuntimeError('Processing queue full; abort without silently replacing frames')
                            report['receiver_queue_max_size'] = max(report['receiver_queue_max_size'], inbox.qsize())
                        if any(time.monotonic() - t > 3 for t in seen.values()):
                            raise RuntimeError('Selected stream missing for >3 seconds')
                report['timed_elapsed_s'] = time.monotonic() - start
            except BaseException as exc:
                report['receiver_errors'].append(repr(exc))
                stop.set()
            finally:
                receiver_done.set()

        receiver = threading.Thread(target=receive, name='rgbd-receive', daemon=False)
        receiver.start()
        sample_due, save_due, progress_due = 1., 2., 30.
        with (output / 'samples.jsonl').open('x', encoding='utf-8') as samples, (output / 'processing.csv').open('x', newline='') as timing_file:
            timing_writer = csv.writer(timing_file)
            timing_writer.writerow(['stream', 'index', 'decode_start_perf_ns', 'decode_end_perf_ns', 'decode_ns', 'receipt_to_decode_end_ns', 'decoded'])
            for meta, im, decode_start, decode_end in ordered_frames(inbox, receiver_done, decode_pool, decode_workers, report, color_decode_stride):
                name = meta['stream']
                if name == 'color':
                    report['color_decoded_frames'] += im is not None
                elif name == 'depth':
                    valid = bool(np.any(im > 0))
                    report['depth_quality']['valid_frames'] += valid
                    report['depth_quality']['zero_frames'] += not valid
                timing_writer.writerow([name, meta['index'], decode_start, decode_end, decode_end-decode_start,
                                        decode_end-meta['host_perf_counter_ns'], im is not None])
                if im is not None:
                    latest[name] = im
                processed[name] += 1
                now = time.monotonic() - start
                if now >= sample_due:
                    sample = {'elapsed_s': now}
                    if 'depth' in latest:
                        sample['depth'] = depth_metrics(latest['depth'])
                    if 'color' in latest:
                        gray = cv2.cvtColor(latest['color'], cv2.COLOR_BGR2GRAY)
                        sample['color'] = {'mean_luma': float(gray.mean()), 'black_fraction': float(np.mean(gray <= 2)),
                                           'white_fraction': float(np.mean(gray >= 253)),
                                           'laplacian_variance': float(cv2.Laplacian(gray, cv2.CV_32F).var())}
                    if 'ir' in latest:
                        sample['ir'] = {'min': int(latest['ir'].min()), 'max': int(latest['ir'].max()),
                                        'mean': float(latest['ir'].mean()), 'zero_fraction': float(np.mean(latest['ir'] == 0))}
                    samples.write(json.dumps(sample) + '\n')
                    report['last_sample'] = sample
                    sample_due = now + 1
                if now >= save_due:
                    if pending_save:
                        if not pending_save.done():
                            raise RuntimeError('Previous snapshot still pending; abort instead of growing save queue')
                        pending_save.result()
                    pending_save = saver.submit(save_snapshot, output, int(now), latest.copy())
                    save_due = now + 60
                if now >= progress_due:
                    progress = {'elapsed_s': round(now, 2), 'received': {k: v.count for k,v in stats.items()},
                                'processed': processed.copy(), 'index_gaps': {k: v.index_gaps for k,v in stats.items()},
                                'depth_quality': report['depth_quality'], 'queue_max_size': report['receiver_queue_max_size']}
                    (output / 'progress.json').write_text(json.dumps(progress, indent=2), encoding='utf-8')
                    print(json.dumps(progress), flush=True)
                    progress_due = now + 30
        receiver.join(timeout=5)
        if receiver.is_alive():
            raise RuntimeError('Receiver did not stop')
        report['status_after'] = status(camera.sdk, camera.device)
        if camera.uvc_clock:
            report['uvc_pts_total_wraps_including_warmup'] = camera.uvc_clock.wraps
        if pending_save:
            pending_save.result()
        save_snapshot(output, int(seconds), latest)
        if report['receiver_errors'] or stop.is_set():
            raise RuntimeError('Receiver error or cancellation; see evidence')
        if any(stats[k].count != processed[k] for k in stats):
            raise RuntimeError('Received/processed frame count mismatch')
        report['status'] = 'completed'
    except BaseException as exc:
        report['status'] = 'failed'
        report['processing_errors'].append(repr(exc))
    finally:
        stop.set()
        decode_pool.shutdown(wait=True, cancel_futures=True)
        saver.shutdown(wait=True, cancel_futures=True)
        if pending_save and not pending_save.cancelled():
            try:
                pending_save.result()
            except BaseException as exc:
                report['processing_errors'].append(f'Snapshot error: {exc!r}')
                report['status'] = 'failed'
        if receiver and receiver.is_alive():
            receiver.join(timeout=5)
        if receiver and receiver.is_alive():
            # SDK receive calls have 200ms timeouts; do not free in-use handles.
            report['cleanup_errors'] = ['Receiver still running; camera close unsafe']
        else:
            report['cleanup_errors'] = camera.close() if camera else []
        report['streams'] = {k: s.summary() for k,s in stats.items()}
        report['processed_frames'] = processed
        report['ended_at_utc'] = datetime.now(timezone.utc).isoformat()
        if report['cleanup_errors']:
            report['status'] = 'failed'
        (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('calibration','status_before','status_after')}, indent=2), flush=True)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--preset', choices=PROFILES, default='both')
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--output', type=Path)
    p.add_argument('--stop-file')
    p.add_argument('--color-decode-stride', type=int, default=1,
                   help='Decode one color frame in N for constrained hosts; sampled validation is explicitly marked')
    p.add_argument('--decode-workers', type=int, default=1,
                   help='1..4 bounded workers; preserve receipt order and decode every RGB frame at stride=1')
    p.add_argument('--uvc-pts', action='store_true',
                   help='Linux Gemini E V4L2 only: descriptor-frequency PTS field; preserves raw SDK timestamps')
    a = p.parse_args()
    if not 1 <= a.seconds <= 1800:
        p.error('seconds must be 1..1800')
    if not 1 <= a.color_decode_stride <= 30:
        p.error('color-decode-stride must be 1..30')
    if not 1 <= a.decode_workers <= 4:
        p.error('decode-workers must be 1..4')
    if not a.execute:
        print(json.dumps({'mode':'offline_preview', 'profiles':PROFILES[a.preset], 'seconds':a.seconds,
                          'queue_overflow':'abort', 'automatic_reconnect':False,
                          'color_decode_stride':a.color_decode_stride, 'decode_workers':a.decode_workers,
                          'sdk_config':str(SDK_CONFIG), 'uvc_pts':a.uvc_pts}, indent=2))
        return 0
    if not a.output:
        p.error('Hardware execution requires --output for a new directory')
    return 0 if capture(a.output, a.preset, a.seconds, a.stop_file, a.color_decode_stride, a.decode_workers, a.uvc_pts)['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
