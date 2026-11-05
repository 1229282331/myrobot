"""Optimized fixed-size quaternion arithmetic against an independent library."""
import math
from pathlib import Path
import sys
import numpy as np
import pytest
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.control_math import multiply,matrix,pose,rotvec,integrate,unit,angle


def reference_quat_matrix(q):
    out=np.zeros(9);mujoco.mju_quat2Mat(out,np.asarray(q)[[3,0,1,2]])
    return out.reshape(3,3)


def reference_euler(values):
    x,y,z=values;sx,cx,sy,cy,sz,cz=np.sin(x),np.cos(x),np.sin(y),np.cos(y),np.sin(z),np.cos(z)
    rx=np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]]);ry=np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]]);rz=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
    return rz@ry@rx


def test_random_hamilton_composition_and_sign_match_independent_rotation():
    rng=np.random.default_rng(104)
    for _ in range(100):
        a,b=rng.normal(size=(2,4));a/=np.linalg.norm(a);b/=np.linalg.norm(b)
        q=multiply(a,b);reference=np.zeros(4)
        mujoco.mju_mulQuat(reference,a[[3,0,1,2]],b[[3,0,1,2]])
        np.testing.assert_allclose(matrix(q),reference_quat_matrix(reference[[1,2,3,0]]),atol=1e-14)
        np.testing.assert_allclose(matrix(multiply(-a,b)),matrix(q),atol=1e-14)


def test_base_and_tool_increments_across_angle_wrap_against_independent_rotation():
    p=np.array([.1,-.2,.4]);raw=[.3,-.4,math.pi-.001];base=reference_euler(raw)
    _,q=pose([100,-200,400,*np.degrees(raw)]);v=np.array([.003,-.002,.001]);w=np.array([.01,-.02,.05]);dt=.05
    # MuJoCo independently converts an axis-angle increment to quaternion.
    iq=np.zeros(4);mujoco.mju_axisAngle2Quat(iq,w/np.linalg.norm(w),np.linalg.norm(w)*dt)
    increment=reference_quat_matrix(iq[[1,2,3,0]])
    for frame in ('base','tool'):
        pp,rr=integrate(p,q,v,w,dt,frame)
        expected=increment@base if frame=='base' else base@increment
        np.testing.assert_allclose(matrix(rr),expected,atol=1e-14)
        np.testing.assert_allclose(pp,p+(v if frame=='base' else base@v)*dt,atol=1e-14)
    for values in ([179.999,-30,180.001],[-180.001,30,-179.999]):
        pp,rr=pose([100,-200,400,*values]);np.testing.assert_allclose(pp,p,atol=1e-14)
        np.testing.assert_allclose(matrix(rr),reference_euler(np.radians(values)),atol=1e-14)


@pytest.mark.parametrize('q',[[0,0,0,0],[float('nan'),0,0,1],[0,0,float('inf'),1],[1,0,0],[1e308]*4])
def test_invalid_quaternions_still_fail_closed(q):
    with pytest.raises(ValueError):unit(q)


@pytest.mark.parametrize('rotation',[1e-9,9.98610685e-8,1.0001e-7,1e-4,math.pi-1e-9,math.pi])
def test_rotation_distance_preserves_small_angles_and_antipodal_equivalence(rotation):
    _,origin=pose([0,0,0,-135.6675,8.3588,-168.13])
    axis=np.array([1.,-2.,3.]);axis/=np.linalg.norm(axis)
    delta=np.zeros(4);mujoco.mju_axisAngle2Quat(delta,axis,rotation)
    expected=np.zeros(4);mujoco.mju_mulQuat(expected,delta,origin[[3,0,1,2]])
    target=expected[[1,2,3,0]]
    assert angle(origin,target)==pytest.approx(rotation,abs=2e-15)
    assert angle(origin,-target)==pytest.approx(rotation,abs=2e-15)
