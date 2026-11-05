import unittest
import numpy as np
import cv2
from tools.grasp_geometry import GeometryError,pose_transform
from tools.grasp_calibration import marker_points,estimate_marker,opencv_distortion

class CalibrationTests(unittest.TestCase):
    def test_marker_pose_against_independent_3d_truth(self):
        pose=pose_transform([15,-20,650,160,8,5]);rv=cv2.Rodrigues(pose[:3,:3])[0]
        k=np.array([[480,0,320],[0,480,180],[0,0,1]],float)
        image,_=cv2.projectPoints(marker_points(100),rv,pose[:3,3],k,np.zeros(5))
        result=estimate_marker(image,k,np.zeros(5),100)
        best=np.asarray(result['pose_candidates'][0]['T_camera_target_mm'])
        np.testing.assert_allclose(best,pose,atol=1e-6)
        self.assertIsNone(result['robot_target'])
    def test_tiny_marker_and_invalid_scale_rejected(self):
        with self.assertRaises(GeometryError):marker_points(0)
        with self.assertRaises(GeometryError):marker_points(float('nan'))
        with self.assertRaises(GeometryError):estimate_marker([[0,0],[20,0],[20,20],[0,20]],np.eye(3),np.zeros(5),100)

    def test_nonzero_rational_distortion_against_projected_truth(self):
        coefficients={'k1':-.019,'k2':.0015,'k3':.0001,'k4':.0002,'k5':-.0001,'k6':.00005,'p1':.00023,'p2':.00024}
        expected=np.array([-.019,.0015,.00023,.00024,.0001,.0002,-.0001,.00005])
        np.testing.assert_allclose(opencv_distortion(coefficients),expected)
        pose=pose_transform([120,-70,560,155,12,8])
        k=np.array([[362,0,321],[0,362,178],[0,0,1]],float)
        pixels,_=cv2.projectPoints(marker_points(111),cv2.Rodrigues(pose[:3,:3])[0],pose[:3,3],k,expected)
        result=estimate_marker(pixels,k,opencv_distortion(coefficients),111)
        np.testing.assert_allclose(result['pose_candidates'][0]['T_camera_target_mm'],pose,atol=1e-5)

if __name__=='__main__':unittest.main()
