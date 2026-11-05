"""SI pose arithmetic shared by the X5 controller; no simulator dependencies."""
import math
import numpy as np

def unit(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all():raise ValueError('Invalid quaternion')
    norm=math.sqrt(float(q@q))
    if norm<1e-12 or not math.isfinite(norm):raise ValueError('Invalid quaternion')
    return q/norm

def multiply(a,b):
    a,b=unit(a),unit(b)
    ax,ay,az,aw=a;bx,by,bz,bw=b
    return unit([aw*bx+ax*bw+ay*bz-az*by,aw*by+ay*bw+az*bx-ax*bz,
                 aw*bz+az*bw+ax*by-ay*bx,aw*bw-ax*bx-ay*by-az*bz])

def matrix(q):
    x,y,z,w=unit(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def rotvec(v):
    v=np.asarray(v,dtype=float);n=math.sqrt(float(v@v))
    if n<1e-12:return unit([*(v/2),1])
    return np.array([*(v*(math.sin(n/2)/n)),math.cos(n/2)])

def angle(a,b):
    # acos(dot) loses precision near identical orientations. Quaternion
    # chord lengths retain tiny rotations without changing the SO(3) angle
    # or accepting the antipodal quaternion as a different orientation.
    a,b=unit(a),unit(b)
    if float(a@b)<0:b=-b
    difference=a-b;total=a+b
    return 4*math.atan2(math.sqrt(float(difference@difference)),math.sqrt(float(total@total)))

def pose(p):
    r,pitch,y=np.radians(p[3:])/2
    sr,cr,sp,cp,sy,cy=math.sin(r),math.cos(r),math.sin(pitch),math.cos(pitch),math.sin(y),math.cos(y)
    return np.asarray(p[:3])/1000,unit([sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy])

def controller_pose(position,q,previous):
    m=matrix(q);pitch=math.asin(float(np.clip(-m[2,0],-1,1)))
    if abs(math.cos(pitch))<1e-5:raise ValueError('Euler representation near singularity')
    r=math.atan2(m[2,1],m[2,2]);y=math.atan2(m[1,0],m[0,0])
    candidates=[np.degrees([r,pitch,y]),np.degrees([r+math.pi,math.pi-pitch,y+math.pi])]
    previous=np.asarray(previous[3:])
    candidates=[c+360*np.round((previous-c)/360) for c in candidates]
    return [*list(np.asarray(position)*1000),*min(candidates,key=lambda c:np.linalg.norm(c-previous)).tolist()]

def integrate(p,q,v,w,dt,frame):
    delta=rotvec(np.asarray(w)*dt)
    if frame=='tool':return np.asarray(p)+matrix(q)@np.asarray(v)*dt,multiply(q,delta)
    return np.asarray(p)+np.asarray(v)*dt,multiply(delta,q)

def slew(current,target,amount):
    delta=np.asarray(target)-current;norm=np.linalg.norm(delta)
    return np.asarray(current)+delta*min(1,amount/max(norm,1e-12))
