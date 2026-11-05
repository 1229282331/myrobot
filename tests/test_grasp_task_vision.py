"""Real prior images, plus explicit failure cases; no camera or arm access."""
import copy,json,tempfile,unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
from tools import grasp_task_vision as vision
from tools.grasp_task_plan import read_profile,HISTORY,scene_input
from tools.grasp_task import digest
from tools.e6_zero import E6Error

def observation(name):
    r=json.loads((HISTORY/name/'report.json').read_text())
    return dict(image=str(HISTORY/name/'0008_color.png'),pose=json.loads((HISTORY/(name+'_observation.json')).read_text())['after']['pose'],calibration=r['calibration'])

class VisionTests(unittest.TestCase):
    def setUp(self):
        self.profile=read_profile();self.contact=observation('force64_contact190_2336');self.probe=observation('lift10_view_2339')
        self.before=observation('open60_view_2310');self.after=observation('final_hover280_2356')
    def test_real_hover_target(self):
        r=vision.target(self.before,self.profile);self.assertLess(r['reprojection_error_px'],8);self.assertGreater(r['area_pixels'],7000);json.dumps(r)
    def test_real_contact_silhouette(self):self.assertEqual(vision.contact(self.contact)['status'],'passed')
    def test_real_10mm_lift(self):
        r=vision.lift(self.contact,self.probe)
        self.assertGreaterEqual(r['table_inliers'],20);self.assertLess(r['bottle_motion_px'],.5);self.assertGreater(r['predicted_table_motion_px'],9);json.dumps(r)
    def test_real_original_location_release(self):self.assertEqual(vision.released(self.before,self.after,self.profile)['status'],'passed')
    def test_no_real_image_motion_is_rejected_even_if_pose_claims_lift(self):
        after=copy.deepcopy(self.contact);after['pose'][2]+=10
        with self.assertRaisesRegex(E6Error,'verified lift'):vision.lift(self.contact,after)
    def test_30mm_is_not_an_accepted_probe(self):
        after=copy.deepcopy(self.probe);after['pose'][2]+=20
        with self.assertRaisesRegex(E6Error,'10mm'):vision.lift(self.contact,after)
    def test_missing_blue_target_or_image_rejected(self):
        blank=np.zeros((360,640,3),np.uint8)
        with patch.object(vision,'image_at',return_value=blank),self.assertRaises(E6Error):vision.target(self.before,self.profile)
        with self.assertRaises(E6Error):vision.image_at(dict(image='absent-image.png'))
    def test_ambiguous_two_blue_candidates_rejected(self):
        a=np.zeros((120,120,3),np.uint8);a[5:25,5:25]=(255,0,0);a[60:90,60:90]=(255,0,0)
        with self.assertRaisesRegex(E6Error,'exactly one'):vision.blue_mask(a,[0,0,120,120])
    def test_zero_intrinsics_and_mirrored_image_rejected(self):
        for field,value in [('fx',0),('width',320)]:
            bad=copy.deepcopy(self.before);bad['calibration']['rgbIntrinsic'][field]=value
            with self.subTest(field=field),self.assertRaises(E6Error):vision.target(bad,self.profile)
        bad=copy.deepcopy(self.before);bad['calibration']['isMirrored']=True
        with self.assertRaises(E6Error):vision.target(bad,self.profile)
    def test_insufficient_texture_and_reverse_flow_failure(self):
        with patch.object(cv2,'goodFeaturesToTrack',return_value=None),self.assertRaisesRegex(E6Error,'texture'):vision.lift(self.contact,self.probe)
        original=cv2.calcOpticalFlowPyrLK;count=[0]
        def flow(*a,**k):
            count[0]+=1
            return (None,None,None) if count[0]==2 else original(*a,**k)
        with patch.object(cv2,'calcOpticalFlowPyrLK',side_effect=flow),self.assertRaisesRegex(E6Error,'Reverse'):vision.lift(self.contact,self.probe)
    def test_release_displacement_rejected(self):
        image=vision.image_at(self.after);shift=np.float32([[1,0,20],[0,1,0]]);moved=cv2.warpAffine(image,shift,(640,360))
        original=vision.image_at
        def at(obs):return moved if obs is self.after else original(obs)
        with patch.object(vision,'image_at',side_effect=at),self.assertRaises(E6Error):vision.released(self.before,self.after,self.profile)
    def test_hover_bbox_keeps_exactly_two_clear_bottom_rows(self):
        # Isolate the exclusive bbox endpoint. Preserve the existing 2-row margin.
        image=np.zeros((360,640,3),np.uint8);mask=np.zeros((360,640),np.uint8)
        center=np.array([345.8,306.05]);stats=[298,246,93,112,7894]
        with patch.object(vision,'image_at',return_value=image),patch.object(vision,'blue_mask',return_value=(mask,center,stats)):
            self.assertEqual(vision.target(self.before,self.profile)['status'],'passed')
        for height in (113,114):
            with self.subTest(height=height),patch.object(vision,'image_at',return_value=image),patch.object(vision,'blue_mask',return_value=(mask,center,[298,246,93,height,7894])),self.assertRaisesRegex(E6Error,'clipped'):
                vision.target(self.before,self.profile)
    def test_old_scene_not_reusable_for_live_preview(self):
        with self.assertRaisesRegex(E6Error,'180 seconds'):scene_input(HISTORY/'final_hover280_2356_observation.json',self.profile)
    def test_clipped_planning_is_explicit_and_does_not_accept_other_target_failures(self):
        run=HISTORY/'final_hover280_2356';report=json.loads((run/'report.json').read_text())
        state=dict(mode=5,q=self.profile['reviewed_hover_q'],pose=self.profile['reviewed_hover_pose'])
        obs=dict(status='static_scene',source_run=str(run.resolve()),camera_report_sha256=digest(run/'report.json'),before=state,after=state)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'obs.json';path.write_text(json.dumps(obs));now=datetime.fromisoformat(report['ended_at_utc'])
            with patch.object(vision,'target',side_effect=E6Error('Bottle silhouette clipped by search ROI or image boundary')):
                with self.assertRaises(E6Error):scene_input(path,self.profile,now)
                _,r=scene_input(path,self.profile,now,allow_clipped_for_view_planning=True)
                self.assertEqual(r['target_check']['status'],'deferred')
            with patch.object(vision,'target',side_effect=E6Error('Bottle differs from the reviewed fixed target')),self.assertRaises(E6Error):
                scene_input(path,self.profile,now,allow_clipped_for_view_planning=True)

if __name__=='__main__':unittest.main()
