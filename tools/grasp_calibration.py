"""Offline ArUco target generation/detection and TCP/hand-eye fitting.

This tool never connects to a robot or writes controller parameters.
T_destination_source maps source coordinates into destination coordinates;
translations are millimetres. For example T_camera_target is target2cam.
"""
import argparse,hashlib,json,math
from pathlib import Path
import cv2
import numpy as np
try:
    from .grasp_geometry import GeometryError,fit_hand_eye,validate_hand_eye,solve_pivot,pose_transform
except ImportError:
    from grasp_geometry import GeometryError,fit_hand_eye,validate_hand_eye,solve_pivot,pose_transform

DICTIONARY=cv2.aruco.DICT_4X4_50
MARKER_ID=17
EDGE_MM=100.0

def generate_target(directory):
    directory.mkdir(parents=True,exist_ok=True)
    marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(DICTIONARY),MARKER_ID,60)
    cells=[]
    for row in range(6):
        for col in range(6):
            if marker[row*10+5,col*10+5]==0:
                cells.append(f'<rect x="{55+col*100/6:.9f}" y="{55+row*100/6:.9f}" width="{100/6:.9f}" height="{100/6:.9f}" fill="black"/>')
    ticks=''.join(f'<path d="M{55+i*10} 175v-5" stroke="black" stroke-width="0.3"/><text x="{55+i*10}" y="181" font-size="3" text-anchor="middle">{i*10}</text>' for i in range(11))
    svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" viewBox="0 0 210 297">
<rect width="210" height="297" fill="white"/>
<g font-family="Arial,Microsoft YaHei,sans-serif" fill="black">
<text x="105" y="25" font-size="7" text-anchor="middle">相机标定图</text>
<text x="105" y="36" font-size="4" text-anchor="middle">黑色外框：100 mm × 100 mm</text>
{''.join(cells)}
<path d="M55 175h100" stroke="black" stroke-width="0.3"/>{ticks}
<text x="105" y="199" font-size="4" text-anchor="middle">A4，打印比例 100%，关闭“适合页面”</text>
<text x="105" y="210" font-size="4" text-anchor="middle">用尺子核对黑色外框边长，平铺并固定</text>
<text x="105" y="221" font-size="4" text-anchor="middle">屏幕显示时也须量尺核对实际边长</text>
</g></svg>'''
    with (directory/'marker_A4_100mm.svg').open('x',encoding='utf-8') as f:f.write(svg)
    html='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>100mm 标定图</title><style>@page{size:A4;margin:0}html,body{margin:0;padding:0;width:210mm;background:white}svg{display:block;width:210mm;height:297mm}</style>'+svg+'</html>'
    with (directory/'print_marker.html').open('x',encoding='utf-8') as f:f.write(html)
    manifest={'dictionary':'DICT_4X4_50','id':MARKER_ID,'black_outer_edge_mm':EDGE_MM,'page_mm':[210,297],'physical_measurement_required':True,'scope':'Nominal printable scale; no printer/screen accuracy assumed','files':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in directory.iterdir() if p.is_file()}}
    with (directory/'target_manifest.json').open('x',encoding='utf-8') as f:json.dump(manifest,f,indent=2)
    return manifest


def generate_screen_target(directory):
    """Portable screen target; CSS pixels are never treated as millimetres."""
    directory.mkdir(parents=True, exist_ok=True)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(DICTIONARY), MARKER_ID, 60)
    cells = ''.join(f'<rect x="{col+1}" y="{row+1}" width="1" height="1"/>'
                    for row in range(6) for col in range(6) if marker[row*10+5,col*10+5] == 0)
    svg = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 8 8" '
           'role="img" aria-label="ArUco ID17 标定图" shape-rendering="crispEdges">'
           '<rect width="8" height="8" fill="white"/><g fill="black">' + cells + '</g></svg>')
    template = (Path(__file__).parent/'templates/tablet_marker.html').read_text(encoding='utf-8')
    outputs = {'marker_screen.svg':svg, 'tablet_marker.html':template.replace('<!-- TARGET_SVG -->',svg)}
    for name, content in outputs.items():
        with (directory/name).open('x', encoding='utf-8') as f:f.write(content)
    large = cv2.resize(marker, (1200,1200), interpolation=cv2.INTER_NEAREST)
    raster = cv2.copyMakeBorder(large,200,200,200,200,cv2.BORDER_CONSTANT,value=255)
    png_path=directory/'marker_screen.png'
    if png_path.exists():raise FileExistsError(png_path)
    if not cv2.imwrite(str(png_path), raster):raise OSError('Unable to save screen target PNG')
    manifest={'dictionary':'DICT_4X4_50','id':MARKER_ID,
              'physical_measurement_required':True,'black_edge_fraction_of_image':.75,
              'scope':'RGB planar target; measure black frame after entering display mode; screen depth is not assumed valid',
              'files':{name:hashlib.sha256((directory/name).read_bytes()).hexdigest()
                       for name in [*outputs,'marker_screen.png']}}
    with (directory/'screen_target_manifest.json').open('x',encoding='utf-8') as f:json.dump(manifest,f,indent=2)
    return manifest

def marker_points(edge_mm):
    if not math.isfinite(edge_mm) or not 30<=edge_mm<=200:raise GeometryError('Physically measured marker edge must be 30..200 mm')
    h=edge_mm/2
    return np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float64)


def opencv_distortion(values):
    """OB distortion fields in OpenCV's rational-model coefficient order."""
    coefficients=np.asarray([values[k] for k in ['k1','k2','p1','p2','k3','k4','k5','k6']],dtype=float)
    if not np.isfinite(coefficients).all():raise GeometryError('Nonfinite camera distortion')
    return coefficients

def estimate_marker(corners,k,distortion,edge_mm):
    c=np.asarray(corners,dtype=np.float64).reshape(4,2)
    if not np.isfinite(c).all() or min(np.linalg.norm(c-np.roll(c,1,axis=0),axis=1))<40:
        raise GeometryError('Marker too small or invalid for reliable pose')
    points=marker_points(edge_mm)
    result=cv2.solvePnPGeneric(points,c,k,distortion,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    poses=[]
    for rv,tv in zip(result[1],result[2]):
        r,_=cv2.Rodrigues(rv);t=tv[:,0]
        if min((points@r.T+t)[:,2])<=0:continue
        projected,_=cv2.projectPoints(points,rv,tv,k,distortion)
        rms=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-c)**2,axis=1))))
        matrix=np.eye(4);matrix[:3,:3]=r;matrix[:3,3]=t
        poses.append({'T_camera_target_mm':matrix.tolist(),'reprojection_rms_px':rms})
    poses.sort(key=lambda v:v['reprojection_rms_px'])
    if not poses or poses[0]['reprojection_rms_px']>1:raise GeometryError('Marker reprojection/cheirality check failed')
    return {'corners_xy':c.tolist(),'pose_candidates':poses,'ambiguity_requires_multiview_check':len(poses)>1,'status':'observation_only','robot_target':None}

def detect(run,second,edge_mm,height_mm=None):
    if height_mm is None or not math.isfinite(height_mm) or abs(height_mm-edge_mm) > .01*edge_mm:
        raise GeometryError('Measure both black-frame edges; the square requires agreement within 1 percent')
    report=json.loads((run/'report.json').read_text(encoding='utf-8'))
    if report['status']!='completed' or 'color' not in report.get('profiles',{}):
        raise GeometryError('Use a successful capture with a calibrated RGB stream')
    if report['calibration'].get('isMirrored') is not False:raise GeometryError('Require an unmirrored camera image')
    intrinsic=report['calibration']['rgbIntrinsic'];dist=opencv_distortion(report['calibration']['rgbDistortion'])
    k=np.array([[intrinsic['fx'],0,intrinsic['cx']],[0,intrinsic['fy'],intrinsic['cy']],[0,0,1]],float)
    if not np.isfinite(k).all() or min(intrinsic['fx'],intrinsic['fy'])<=0:raise GeometryError('Camera intrinsics are missing')
    img=cv2.imread(str(run/f'{second:04d}_color.png'))
    if img is None or img.shape[:2]!=(intrinsic['height'],intrinsic['width']):raise GeometryError('Image/calibration mismatch')
    profile=report['profiles']['color']
    if (profile['width'],profile['height'])!=(intrinsic['width'],intrinsic['height']):
        raise GeometryError('RGB profile/calibration mismatch')
    parameters=cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize=3
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY),parameters)
    corners,ids,_=detector.detectMarkers(img)
    if ids is None or np.sum(ids[:,0]==MARKER_ID)!=1:raise GeometryError('Exactly one marker ID17 must be visible')
    c=corners[np.flatnonzero(ids[:,0]==MARKER_ID)[0]].reshape(4,2)
    if np.min(c)<5 or np.max(c[:,0])>img.shape[1]-6 or np.max(c[:,1])>img.shape[0]-6:raise GeometryError('Marker clipped or too close to image boundary')
    image_path=run/f'{second:04d}_color.png'
    return {**estimate_marker(c,k,dist,(edge_mm+height_mm)/2),'marker_id':MARKER_ID,
            'measured_width_height_mm':[edge_mm,height_mm],'source_run':str(run),
            'image_sha256':hashlib.sha256(image_path.read_bytes()).hexdigest(),
            'second_label':second,'camera_model':intrinsic,'distortion_opencv':dist.tolist(),
            'source_preset':report['preset'],'alignment_mode':report.get('alignment_mode'),
            'transform_convention':'T_camera_target_mm maps target points into camera coordinates'}

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True)
    a=sub.add_parser('target');a.add_argument('--output-dir',type=Path,required=True)
    a=sub.add_parser('target-screen');a.add_argument('--output-dir',type=Path,required=True)
    a=sub.add_parser('detect');a.add_argument('--run',type=Path,required=True);a.add_argument('--second',type=int,required=True);a.add_argument('--measured-edge-mm',type=float,required=True);a.add_argument('--measured-height-mm',type=float,required=True);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('tcp');a.add_argument('--poses',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('hand-eye');a.add_argument('--observations',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('validate-hand-eye');a.add_argument('--fit',type=Path,required=True);a.add_argument('--observations',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.mode=='target':result=generate_target(args.output_dir)
    elif args.mode=='target-screen':result=generate_screen_target(args.output_dir)
    elif args.mode=='detect':result=detect(args.run,args.second,args.measured_edge_mm,args.measured_height_mm)
    elif args.mode=='tcp':
        data=json.loads(args.poses.read_text());result=solve_pivot([pose_transform(v) for v in data['same_fixed_point_contact_poses_xyzmm_rxyzdeg']])
    else:
        data=json.loads(args.observations.read_text());observations=data['observations']
        ids=[v['image_sha256'] for v in observations]
        if len(set(ids))!=len(ids) or any(len(h)!=64 or any(c not in '0123456789abcdef' for c in h) for h in ids):
            raise GeometryError('Require distinct source image SHA256 hashes')
        gs=[pose_transform(v['flange_pose_xyzmm_rxyzdeg']) for v in observations]
        cs=[v['T_camera_target_mm'] for v in observations]
        if args.mode=='hand-eye':
            result=fit_hand_eye(gs,cs);result['fit_image_sha256']=ids
        else:
            fit=json.loads(args.fit.read_text())
            if not fit.get('fit_image_sha256') or set(ids)&set(fit['fit_image_sha256']):
                raise GeometryError('Held-out images must be separate from fitting images')
            result=validate_hand_eye(fit,gs,cs)
            result['validation_image_sha256']=ids
            result['fit_file_sha256']=hashlib.sha256(args.fit.read_bytes()).hexdigest()
    if args.mode not in ('target','target-screen'):
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
