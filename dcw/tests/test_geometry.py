import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from geometry_check import deproject, plane_metrics


class Geometry(unittest.TestCase):
    def test_millimeters_to_meters_and_axes(self):
        k=dict(width=3,height=3,fx=100,fy=100,cx=1,cy=1)
        xyz=deproject(np.full((3,3),1000),k)
        np.testing.assert_allclose(xyz[1,1],[0,0,1])
        np.testing.assert_allclose(xyz[2,2],[.01,.01,1])

    def test_plane_ignores_invalid_zero_and_requires_matching_intrinsics(self):
        mm=np.full((20,20),500,dtype=np.float32);mm[0,0]=0
        k=dict(width=20,height=20,fx=100,fy=100,cx=10,cy=10)
        result=plane_metrics(deproject(mm,k),mm,[0,0,20,20])
        self.assertEqual(result['points'],399)
        self.assertAlmostEqual(result['plane_rms_residual_mm'],0)
        self.assertEqual(result['median_z_mm'],500)
        with self.assertRaises(ValueError):
            deproject(mm,{**k,'width':19})


if __name__=='__main__':
    unittest.main()
