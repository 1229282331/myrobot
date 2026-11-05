"""Observation CLI under fake camera/robot only; no CAN ownership."""
import contextlib,io,json,signal,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from tools import grasp_task_scene as scene,grasp_task_hardware as hardware
from tools.grasp_task_plan import read_profile
from tools.grasp_task import digest
from tests.test_grasp_task_hardware import FakeRobot

class SceneTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        for name in ('socket.create_connection','socket.socket','subprocess.Popen','subprocess.run'):
            p=patch(name,side_effect=AssertionError('No hardware in observation tests'));p.start();self.addCleanup(p.stop)
    def test_default_is_offline(self):
        stream=io.StringIO()
        with patch.object(sys,'argv',['scene']),contextlib.redirect_stdout(stream):code=scene.main()
        self.assertEqual(code,0);self.assertEqual(json.loads(stream.getvalue())['hardware_connections'],0)
    def fake_capture(self,failure=False):
        profile=read_profile();runtime=self.root/'rdk_healthcheck.py';runtime.write_text('fake camera')
        config=self.root/'config.xml';config.write_text('fake configuration')
        profile['camera'].update(tools_dir=str(self.root),healthcheck_sha256=digest(runtime),sdk_config=str(config),sdk_config_sha256=digest(config))
        report=self.root/'report.json';report.write_text('{}');raw=FakeRobot();state=dict(mode=5,q=profile['reviewed_hover_q'],pose=profile['reviewed_hover_pose'])
        obs=dict(robot_before=state,robot_after=state,camera_report=str(report),camera_report_sha256=digest(report))
        observe=patch.object(hardware.HardwareBackend,'observe',side_effect=OSError('Injected capture failure')) if failure else patch.object(hardware.HardwareBackend,'observe',return_value=obs)
        with patch.object(sys,'argv',['scene','--capture','--output',str(self.root/'run')]),patch.object(scene.sys,'platform','linux'),patch.object(scene,'read_profile',return_value=profile),patch.object(hardware,'ObservationDashboard',return_value=raw),patch.object(scene.signal,'SIGHUP',1,create=True),patch.object(scene.signal,'getsignal',return_value=signal.SIG_DFL),patch.object(scene.signal,'signal'),patch.object(hardware.grip.can,'configure') as configure,observe,contextlib.redirect_stdout(io.StringIO()):code=scene.main()
        configure.assert_not_called();self.assertTrue(raw.closed)
        return code,raw
    def test_capture_saves_planner_input_and_has_no_can_or_movement(self):
        code,raw=self.fake_capture();self.assertEqual(code,0);self.assertEqual(raw.commands,[])
        self.assertEqual(json.loads((self.root/'run/scene_observation.json').read_text())['status'],'static_scene')
    def test_capture_failure_stops_once_without_can(self):
        code,raw=self.fake_capture(failure=True);self.assertEqual(code,1);self.assertEqual(raw.commands,['Stop()'])

if __name__=='__main__':unittest.main()
