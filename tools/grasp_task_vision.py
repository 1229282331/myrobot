"""Offline fixed-blue-bottle gates; zero depth never becomes a target."""
import json,math
from pathlib import Path
import cv2,numpy as np
from .grasp_geometry import pose_transform,checked_transform
from .e6_zero import E6Error
CONTACT_ROI=[260,310,195,170]  # Fixed station: bottle between fingers; excludes left finger's blue reflection.

def blue_mask(image,roi):
    x,y,w,h=roi
    if x<0 or y<0 or w<1 or h<1 or x+w>image.shape[1] or y+h>image.shape[0]:raise E6Error('Visual ROI outside image')
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,np.array([85,50,20]),np.array([125,255,255]))
    region=np.zeros(mask.shape,np.uint8);region[y:y+h,x:x+w]=255;mask=cv2.bitwise_and(mask,region)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    valid=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=80]
    if len(valid)!=1:raise E6Error('Require exactly one visible blue candidate in the fixed ROI')
    i=valid[0]
    return (labels==i).astype(np.uint8)*255,centers[i],stats[i].tolist()

def image_at(obs):
    path=Path(obs['image'])
    if not path.is_file():raise E6Error('Observation image missing')
    image=cv2.imread(str(path))
    if image is None:raise E6Error('Observation image missing or undecodable')
    return image

def dice(a,b):return float(2*np.logical_and(a>0,b>0).sum()/max(1,(a>0).sum()+(b>0).sum()))

def contact_mask(image):
    mask,center,stats=blue_mask(image,CONTACT_ROI)
    if not 8000<=stats[4]<=24000 or not (320<=center[0]<=390 and 410<=center[1]<=460):
        raise E6Error('Contact bottle silhouette differs from the reviewed station')
    return mask,center,stats

def target(obs,bundle):
    image=image_at(obs);cal=obs['calibration'];i=cal['rgbIntrinsic']
    if cal.get('isMirrored') or image.shape[:2]!=(i['height'],i['width']) or min(i['fx'],i['fy'])<=0:raise E6Error('Uncalibrated or mirrored RGB observation')
    hand=checked_transform(bundle['T_flange_camera_mm']);camera=pose_transform(obs['pose'])@hand
    point=(np.linalg.inv(camera)@np.r_[bundle['target_base_mm'],1])[:3]
    if point[2]<=0:raise E6Error('Target is behind camera')
    k=np.array([[i['fx'],0,i['cx']],[0,i['fy'],i['cy']],[0,0,1.]])
    d=cal['rgbDistortion'];dist=np.array([d[n] for n in ['k1','k2','p1','p2','k3','k4','k5','k6']])
    expected=cv2.projectPoints(point[None,:],np.zeros(3),np.zeros(3),k,dist)[0][0,0]
    # Segment the whole blue silhouette. A small ROI clips a large bottle and
    # can hide a displacement by pinning its centroid to the ROI centre.
    x,y=(int(v) for v in np.rint(expected));top=y-110
    roi=[x-110,top,220,image.shape[0]-top]
    mask,center,stats=blue_mask(image,roi);error=float(np.linalg.norm(center-expected))
    bx,by,bw,bh,area=stats
    # OpenCV bbox width/height include the first pixel; x+w/y+h are exclusive.
    # Require two clear image rows below the silhouette, as on the other edges.
    if bx<=roi[0]+1 or bx+bw>=roi[0]+roi[2]-1 or by<=top+1 or by+bh-1>=image.shape[0]-2:
        raise E6Error('Bottle silhouette clipped by search ROI or image boundary')
    if error>8 or not 5000<=area<=12000:raise E6Error('Bottle differs from the reviewed fixed target')
    return dict(status='passed',scope='Fixed-workcell RGB location consistency; no new 3D/TCP target generated',expected_pixel_xy=expected.tolist(),blue_centroid_xy=center.tolist(),reprojection_error_px=error,roi=roi,area_pixels=stats[4])

def contact(obs):
    image=image_at(obs)
    if image.shape!=(480,640,3):raise E6Error('Contact evidence requires the matched 640x480 preview')
    _,center,stats=contact_mask(image)
    return dict(status='passed',scope='Visible blue bottle candidate only; not proof of lifting',centroid_xy=center.tolist(),bbox_xywh_area=stats)

def lift(before,after):
    a,b=image_at(before),image_at(after)
    if a.shape!=(480,640,3) or b.shape!=a.shape:raise E6Error('Lift evidence profile mismatch')
    dz=after['pose'][2]-before['pose'][2]
    if not 9.5<=dz<=10.5:raise E6Error('Visual lift gate requires the 10mm probe')
    m0,c0,_=contact_mask(a);m1,c1,_=contact_mask(b)
    gray=[cv2.cvtColor(im,cv2.COLOR_BGR2GRAY) for im in [a,b]]
    table=np.zeros((480,640),np.uint8);table[20:330,:610]=255
    pts=cv2.goodFeaturesToTrack(gray[0],500,.015,6,mask=table)
    if pts is None or len(pts)<20:raise E6Error('Insufficient table texture for lift verification')
    scale=(before['pose'][2]-38+22)/(after['pose'][2]-38+22);center=np.array([320,240],np.float32)
    seed=(pts-center)*scale+center
    nxt,status,_=cv2.calcOpticalFlowPyrLK(gray[0],gray[1],pts,seed,winSize=(41,41),maxLevel=4,flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    if nxt is None or status is None:raise E6Error('Table optical flow failed')
    back,s2,_=cv2.calcOpticalFlowPyrLK(gray[1],gray[0],nxt,pts.copy(),winSize=(41,41),maxLevel=4,flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    if back is None or s2 is None:raise E6Error('Reverse table optical flow failed')
    valid=(status[:,0]!=0)&(s2[:,0]!=0)&(np.linalg.norm(pts[:,0]-back[:,0],axis=1)<.7)
    src,dst=pts[valid,0],nxt[valid,0]
    if len(src)<20:raise E6Error('Insufficient bidirectional table matches')
    h,inliers=cv2.findHomography(src,dst,cv2.RANSAC,1.)
    if h is None or inliers is None or int(inliers.sum())<20:raise E6Error('Insufficient table inliers; no lift confirmation')
    pred=cv2.perspectiveTransform(src[None,:,:],h)[0];res=np.linalg.norm(pred-dst,axis=1)[inliers[:,0]!=0]
    table_center=cv2.perspectiveTransform(np.array(c0,dtype=np.float32)[None,None,:],h)[0,0]
    observed=float(np.linalg.norm(c1-c0));table_motion=float(np.linalg.norm(table_center-c0));same=dice(m0,m1);warped=dice(cv2.warpPerspective(m0,h,(640,480)),m1)
    if not np.isfinite([observed,table_motion,*res]).all():raise E6Error('Nonfinite table motion estimate')
    if observed>2 or table_motion<3 or float(np.median(res))>.7 or same<.9 or same-warped<.05:
        raise E6Error('Bottle movement does not support a verified lift')
    return dict(status='passed',scope='Relative image-motion evidence plus contact guard; no direct bottle-bottom clearance measurement',table_inliers=int(inliers.sum()),table_median_residual_px=float(np.median(res)),bottle_motion_px=observed,predicted_table_motion_px=table_motion,identity_dice=same,table_warp_dice=warped)

def released(before,after,bundle):
    check=target(after,bundle);a,b=image_at(before),image_at(after)
    initial=target(before,bundle)
    if a.shape!=b.shape:raise E6Error('Release comparison profile changed')
    roi=initial['roi'];m0,c0,_=blue_mask(a,roi);m1,c1,_=blue_mask(b,roi)
    motion=float(np.linalg.norm(c1-c0));overlap=dice(m0,m1)
    if motion>8 or overlap<.85:raise E6Error('Released bottle differs from the original location')
    check.update(centroid_difference_px=motion,identity_dice=overlap,scope='Original-location return image plus verified open feedback; no absolute placement metrology')
    return check
