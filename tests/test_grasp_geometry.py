"""Geometry failure cases and independent synthetic ground truth; no hardware."""
import unittest
import numpy as np
from tools.grasp_geometry import pose_transform,checked_transform,deproject_mm,fit_plane,solve_pivot,fit_hand_eye,validate_hand_eye,intersect_plane_pixels,GeometryError

class GeometryTests(unittest.TestCase):
    def test_invalid_rotation_and_zero_depth(self):
        a=np.eye(4);a[0,0]=-1
        with self.assertRaises(GeometryError):checked_transform(a)
        p=deproject_mm(np.array([[0,1000]]),dict(width=2,height=1,fx=100,fy=100,cx=0,cy=0))
        self.assertTrue(np.isnan(p[0,0]).all());np.testing.assert_allclose(p[0,1],[10,0,1000])
    def test_plane_rejects_nonplanar_and_collinear_data(self):
        rng=np.random.default_rng(4)
        with self.assertRaises(GeometryError):fit_plane(rng.normal(0,100,(500,3)))
        p=np.column_stack([np.arange(500),np.arange(500),np.zeros(500)])
        with self.assertRaises(GeometryError):fit_plane(p)
    def test_plane_preserves_object_outliers(self):
        rng=np.random.default_rng(5);xy=rng.uniform(-100,100,(1000,2));z=rng.normal(500,.5,1000);z[:150]=450
        result=fit_plane(np.column_stack([xy,z]))
        self.assertGreater(result['support'],.8);self.assertLess(result['support'],.9)
        np.testing.assert_allclose(result['normal'],[0,0,-1],atol=.003);self.assertAlmostEqual(result['offset_mm'],500,delta=.5)
    def poses(self):
        return [pose_transform([50+i*7,-100+i*2,350+i*3,*r]) for i,r in enumerate([(0,0,0),(20,0,0),(0,25,0),(0,0,20),(-20,15,0),(15,-20,10),(-10,-15,-20),(20,25,-15)])]
    def test_pivot_recovers_known_tip(self):
        tcp=np.array([3,-4,155]);point=np.array([100,200,300]);poses=self.poses()
        for t in poses:t[:3,3]=point-t[:3,:3]@tcp
        result=solve_pivot(poses)
        np.testing.assert_allclose(result['flange_to_tcp_mm'],tcp,atol=1e-7)
        self.assertFalse(result['validated_for_motion'])
    def test_single_axis_pivot_is_unidentifiable(self):
        poses=[pose_transform([0,0,100,0,0,x]) for x in range(0,80,10)]
        with self.assertRaises(GeometryError):solve_pivot(poses)
    def test_inconsistent_pivot_contacts_rejected(self):
        tcp=np.array([3,-4,155]);point=np.array([100,200,300]);poses=self.poses()
        for t in poses:t[:3,3]=point-t[:3,:3]@tcp
        poses[-1][:3,3]+=[30,-20,15]
        with self.assertRaises(GeometryError):solve_pivot(poses)
    def test_handeye_recovers_known_mount(self):
        x=pose_transform([8,-35,28,5,0,4]);target=pose_transform([100,-250,40,0,0,0]);gs=self.poses();cs=[np.linalg.inv(g@x)@target for g in gs]
        result=fit_hand_eye(gs,cs)
        np.testing.assert_allclose(result['T_flange_camera_mm'],x,atol=1e-6)
        self.assertFalse(result['validated_for_motion'])
    def test_handeye_parallel_axis_and_moving_target_rejected(self):
        x=pose_transform([8,-35,28,5,0,4]);target=pose_transform([100,-250,40,0,0,0])
        gs=[pose_transform([50,-100,350,0,0,i*3]) for i in range(8)];cs=[np.linalg.inv(g@x)@target for g in gs]
        with self.assertRaises(GeometryError):fit_hand_eye(gs,cs)
        gs=self.poses();cs=[np.linalg.inv(g@x)@target for g in gs];cs[-1][:3,3]+=[30,20,25]
        with self.assertRaises(GeometryError):fit_hand_eye(gs,cs)

    def test_heldout_views_detect_mount_or_target_change(self):
        x=pose_transform([8,-35,28,5,0,4]);target=pose_transform([100,-250,40,0,0,0])
        gs=self.poses();cs=[np.linalg.inv(g@x)@target for g in gs]
        fit=fit_hand_eye(gs,cs)
        held=[pose_transform([60,-100,330,*r]) for r in [(12,8,-5),(-12,20,8),(8,-10,15)]]
        observed=[np.linalg.inv(g@x)@target for g in held]
        result=validate_hand_eye(fit,held,observed)
        self.assertTrue(result['independent_consistency_passed'])
        self.assertFalse(result['validated_for_motion'])
        shifted=x.copy();shifted[:3,3]+=[8,0,0]
        changed=[np.linalg.inv(g@shifted)@target for g in held]
        with self.assertRaises(GeometryError):validate_hand_eye(fit,held,changed)
        with self.assertRaises(GeometryError):validate_hand_eye(fit,[held[0]]*3,[observed[0]]*3)

    def test_table_contact_ray_intersection_and_invalid_geometry(self):
        k=dict(width=640,height=360,fx=500,fy=500,cx=320,cy=180)
        point=np.array([25,-30,600.])
        pixel=[[320+500*point[0]/point[2],180+500*point[1]/point[2]]]
        np.testing.assert_allclose(intersect_plane_pixels(pixel,k,[0,0,-1],600),point[None,:])
        with self.assertRaises(GeometryError):intersect_plane_pixels([[320,180]],k,[1,0,0],100)
        with self.assertRaises(GeometryError):intersect_plane_pixels(pixel,k,[0,0,-1],-600)
        with self.assertRaises(GeometryError):intersect_plane_pixels([[640,180]],k,[0,0,-1],600)

if __name__=='__main__':unittest.main()
