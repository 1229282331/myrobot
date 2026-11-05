"""Offline D2C table plane and above-plane candidate extraction.

Returns camera-frame candidates only; never sends a robot/gripper command.
"""
import argparse,json,sys
from pathlib import Path
import cv2
import numpy as np
try:
    from .grasp_geometry import deproject_mm,fit_plane,GeometryError
except ImportError:
    from grasp_geometry import deproject_mm,fit_plane,GeometryError

def roi_mask(shape,roi):
    x,y,w,h=roi
    if min(x,y)<0 or min(w,h)<1 or x+w>shape[1] or y+h>shape[0]:raise GeometryError('ROI outside image')
    mask=np.zeros(shape,bool);mask[y:y+h,x:x+w]=True;return mask

def analyze(depth,intrinsic,plane_roi,search_roi):
    xyz=deproject_mm(depth,intrinsic);valid=np.isfinite(xyz).all(axis=2)
    plane_mask=roi_mask(depth.shape,plane_roi)&valid
    if plane_mask.sum()/roi_mask(depth.shape,plane_roi).sum()<.8:raise GeometryError('Plane ROI lacks sufficient measured depth')
    plane=fit_plane(xyz[plane_mask]);n=np.array(plane['normal']);height=xyz@n+plane['offset_mm']
    candidates_mask=(valid&roi_mask(depth.shape,search_roi)&(height>=8)&(height<=100)).astype('uint8')
    count,labels,stats,_=cv2.connectedComponentsWithStats(candidates_mask,8)
    candidates=[]
    for label in range(1,count):
        if stats[label,cv2.CC_STAT_AREA]<30:continue
        mask=labels==label;points=xyz[mask];center=np.median(points,axis=0)
        candidates.append({'bbox_xywh':stats[label,:4].tolist(),'measured_pixels':int(mask.sum()),'centroid_camera_mm':center.tolist(),'height_p10_p50_p90_mm':np.percentile(height[mask],[10,50,90]).tolist(),'extent_camera_xyz_mm':(np.percentile(points,95,axis=0)-np.percentile(points,5,axis=0)).tolist(),'note':'Candidate may be clutter or registration noise; requires semantic review and independent geometry validation'})
    return {'plane_roi_xywh':plane_roi,'search_roi_xywh':search_roi,'plane':plane,'candidates':candidates,'target_selected':False,'robot_target':None,'invalid_handling':'Zero depth excluded; no filling','scope':'Camera-frame diagnostics only; no gripper TCP/hand-eye or motion authorization'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--second',type=int,required=True)
    p.add_argument('--plane-roi',nargs=4,type=int,required=True);p.add_argument('--search-roi',nargs=4,type=int,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=json.loads((a.run/'report.json').read_text(encoding='utf-8'))
    if r.get('status')!='completed' or r['preset']!='d2c':p.error('Require a successful native D2C capture')
    c=r['calibration']
    if c['isMirrored'] or any(abs(v)>1e-12 for v in c['depthDistortion'].values()):p.error('Mirroring/distortion not supported')
    if any(abs(c['depthIntrinsic'][k]-c['rgbIntrinsic'][k])>1e-6 for k in ['fx','fy','cx','cy','width','height']):p.error('RGB/depth camera models disagree')
    d=np.load(a.run/f'{a.second:04d}_depth_mm.npy')
    try:result=analyze(d,c['depthIntrinsic'],a.plane_roi,a.search_roi)
    except GeometryError as exc:result={'status':'rejected','error':str(exc),'robot_target':None}
    else:result['status']='analyzed'
    result['source_run']=str(a.run);result['second_label']=a.second
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False,indent=2));return int(result['status']=='rejected')
if __name__=='__main__':raise SystemExit(main())
