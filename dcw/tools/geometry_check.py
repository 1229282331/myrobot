"""Offline native depth point cloud and selected flat-surface diagnostics.

Zero depth is excluded. Plane residual measures observed flatness, not accuracy.
No calibration parameters are fitted or written to the camera.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def deproject(mm, intrinsic):
    if mm.shape != (intrinsic['height'], intrinsic['width']):
        raise ValueError('Depth dimensions do not match calibration')
    if not np.isfinite(mm).all() or np.any(mm < 0):
        raise ValueError('Depth contains invalid numbers')
    if not all(np.isfinite(intrinsic[k]) for k in ('fx', 'fy', 'cx', 'cy')) or min(intrinsic['fx'], intrinsic['fy']) <= 0:
        raise ValueError('Invalid camera intrinsics')
    v, u = np.indices(mm.shape, dtype=np.float32)
    z = mm.astype(np.float32) / 1000
    return np.stack([(u-intrinsic['cx'])/intrinsic['fx']*z,
                     (v-intrinsic['cy'])/intrinsic['fy']*z, z], axis=-1)


def plane_metrics(xyz, mm, roi):
    x, y, w, h = roi
    if min(x,y) < 0 or min(w,h) < 1 or x+w > mm.shape[1] or y+h > mm.shape[0]:
        raise ValueError('ROI outside image')
    depth = mm[y:y+h, x:x+w]
    valid = depth > 0
    pts = xyz[y:y+h, x:x+w][valid].astype(np.float64) * 1000
    if len(pts) < 100:
        raise ValueError('Too few valid ROI points')
    center = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts-center, full_matrices=False)
    normal = vh[-1]
    residual = (pts-center) @ normal
    return {'roi_xywh': roi, 'valid_fraction': float(valid.mean()), 'median_z_mm': float(np.median(depth[valid])),
            'plane_rms_residual_mm': float(np.sqrt(np.mean(residual**2))),
            'plane_abs_residual_p95_mm': float(np.percentile(np.abs(residual),95)),
            'plane_normal_xyz': normal.tolist(), 'points': len(pts),
            'scope': 'All valid ROI points, untrimmed plane fit. Assumes selected scene surface is flat; not a distance calibration.'}


def export(run, second, roi=None):
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    calibration = report['calibration']
    if calibration['isMirrored'] or any(abs(v)>1e-12 for v in calibration['depthDistortion'].values()):
        raise ValueError('This exporter requires unmirrored depth and zero depth distortion')
    mm = np.load(run/f'{second:04d}_depth_mm.npy')
    xyz = deproject(mm, calibration['depthIntrinsic'])
    points = np.asarray(xyz[mm>0], dtype='<f4')
    target = run/f'{second:04d}_points_m.ply'
    header = ('ply\nformat binary_little_endian 1.0\ncomment unit meters; x right, y down, z forward\n'
              f'element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
    with target.open('wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.tobytes())
    result = {'source_run': run.name, 'snapshot_second_label': second, 'points': len(points),
              'coordinate_frame': 'color optical frame (hardware D2C)' if report['preset']=='d2c' else 'depth optical frame',
              'unit': 'meter', 'axis': 'x right, y down, z forward', 'ply': target.name,
              'valid_fraction': float(np.mean(mm>0)), 'invalid_handling': 'zero depth excluded, no hole filling',
              'bounds_xyz_m': [points.min(axis=0).tolist(),points.max(axis=0).tolist()] if len(points) else None}
    if roi:
        result['plane'] = plane_metrics(xyz, mm, roi)
    (run/f'{second:04d}_geometry.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--second',type=int,default=2)
    parser.add_argument('--plane-roi',type=int,nargs=4,metavar=('X','Y','W','H'))
    args=parser.parse_args()
    print(json.dumps(export(args.run,args.second,args.plane_roi),indent=2))
