"""Hardware wrappers under fake devices only: Stop and resource failures."""
import copy,json,tempfile,unittest,signal,types,sys
from pathlib import Path
from unittest.mock import Mock,patch
from tools import grasp_task as task
from tools import grasp_task_hardware as h
from tools.grasp_task_plan import example_bundle,utc

class FakeRobot:
    def __init__(self,mode=5):self.commands=[];self.mode=mode;self.closed=False
    def __enter__(self):return self
    def __exit__(self,*_):self.closed=True
    def command(self,cmd,**kwargs):
        self.commands.append(cmd)
        if cmd=='RobotMode()':return str(self.mode)
        if cmd=='GetAngle()':return '-66.5019,-74.9236,42.5551,-57.6311,90,9.1974'
        if cmd=='GetErrorID()':return '[]'
        if cmd.startswith('GetPose'):return '72.6302,-370.9563,280.0005,-179.9996,.0001,-165.6993'
        return ''

class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.j=task.Journal(self.root/'run','test');self.addCleanup(self.j.close)
        for name in ('socket.create_connection','socket.socket','subprocess.Popen','subprocess.run'):
            p=patch(name,side_effect=AssertionError('No hardware calls in tests'));p.start();self.addCleanup(p.stop)
    def test_logging_failure_cannot_block_raw_stop(self):
        raw=FakeRobot();wrapper=h.RecordingRobot(raw,self.j)
        self.j.write=Mock(side_effect=OSError('Disk full'))
        wrapper.command('Stop()');wrapper.command('Stop()');self.assertEqual(raw.commands,['Stop()'])
    def test_stop_timeout_is_not_retried_or_reported_acknowledged(self):
        raw=FakeRobot();raw.command=Mock(side_effect=TimeoutError('Stop acknowledgement missing'))
        wrapper=h.RecordingRobot(raw,self.j)
        with self.assertRaises(TimeoutError):wrapper.command('Stop()')
        with self.assertRaises(task.E6Error):wrapper.command('Stop()')
        self.assertEqual(raw.command.call_count,1);self.assertIsNone(wrapper.stop_result)
    def test_nonreviewed_moves_and_settings_rejected_without_sending(self):
        raw=FakeRobot();wrapper=h.RecordingRobot(raw,self.j)
        for cmd in ('EnableRobot()','DisableRobot()','ClearError()','Continue()','LoadSwitch(1)','MovJ(joint={0,0,0,0,0,0},a=1,v=1,cp=0)'):
            with self.subTest(cmd=cmd),self.assertRaises(task.E6Error):wrapper.command(cmd)
        self.assertEqual(raw.commands,[])
    def test_state_log_has_robot_mode_separate_from_task_mode(self):
        wrapper=h.RecordingRobot(FakeRobot(),self.j)
        h.read_state(wrapper);wrapper.command('GetPose(user=0,tool=0)')
        records=[json.loads(x) for x in (self.j.root/'events.jsonl').read_text().splitlines()]
        r=records[-1];self.assertEqual(r['kind'],'robot_state');self.assertEqual(r['robot_mode'],5);self.assertEqual(r['mode'],'test')
    def test_send_logging_failure_prevents_movement(self):
        raw=FakeRobot();cmd='reviewed move';wrapper=h.RecordingRobot(raw,self.j,[cmd])
        self.j.write=Mock(side_effect=OSError('Disk full'))
        with self.assertRaises(OSError):wrapper.command(cmd)
        self.assertEqual(raw.commands,[])
    def test_reply_logging_failure_still_allows_stop(self):
        raw=FakeRobot();wrapper=h.RecordingRobot(raw,self.j,['reviewed move']);write=self.j.write
        def log(kind,**data):
            if kind=='robot_reply':raise OSError('Disk full after send')
            return write(kind,**data)
        self.j.write=log
        with self.assertRaises(OSError):wrapper.command('reviewed move')
        wrapper.command('Stop()');self.assertEqual(raw.commands,['reviewed move','Stop()'])
    def test_gripper_log_failure_does_not_send_target(self):
        channel=Mock();recording=h.RecordingChannel(channel,self.j);self.j.write=Mock(side_effect=OSError('Disk full'))
        with self.assertRaises(OSError):recording.exchange(b'12345678')
        channel.exchange.assert_not_called()
    def test_partial_preflight_cleanup_stops_and_closes_robot(self):
        b=example_bundle();b['review'].update(kind='live_preview',sampling='controller_fk',created_utc=utc(),clearance_checked=True,clearance_checked_utc=utc(),clearance_note='Fake test only',source_sha256=task.source_manifest())
        camera_dir=self.root/'tools';camera_dir.mkdir();(camera_dir/'rdk_healthcheck.py').write_text('fake only')
        sdk=self.root/'config.xml';sdk.write_text('fake only')
        b['camera'].update(tools_dir=str(camera_dir),sdk_config=str(sdk),healthcheck_sha256=task.digest(camera_dir/'rdk_healthcheck.py'),sdk_config_sha256=task.digest(sdk))
        raw=FakeRobot(mode=11);backend=h.HardwareBackend(self.j)
        with patch.object(h.sys,'platform','linux'),patch.object(h,'ObservationDashboard',return_value=raw),patch.object(h.signal,'SIGHUP',1,create=True),patch.object(h.signal,'getsignal',return_value=signal.SIG_DFL),patch.object(h.signal,'signal'):
            result=task.TaskRunner(b,backend,self.j).run()
        self.assertEqual(result['status'],'aborted');self.assertEqual(raw.commands.count('Stop()'),1);self.assertTrue(raw.closed)
        self.assertIsNone(backend.channel)
    def test_cleanup_continues_after_can_channel_close_error(self):
        raw=FakeRobot();backend=h.HardwareBackend(self.j);backend.raw_robot=raw;backend.robot=h.RecordingRobot(raw,self.j)
        backend.raw_channel=Mock();backend.raw_channel.close.side_effect=OSError('Failed CAN socket close')
        backend.lock=Mock()
        with self.assertRaises(task.E6Error):backend.close()
        self.assertEqual(raw.commands,['Stop()']);self.assertTrue(raw.closed);backend.lock.close.assert_called_once()
    def test_can_initialization_failure_releases_lock_and_owned_interface(self):
        b=example_bundle();b['review'].update(kind='live_preview',sampling='controller_fk',created_utc=utc(),clearance_checked=True,clearance_checked_utc=utc(),clearance_note='Fake test only',source_sha256=task.source_manifest())
        camera_dir=self.root/'camera';camera_dir.mkdir();runtime=camera_dir/'rdk_healthcheck.py';runtime.write_text('fake')
        sdk=self.root/'sdk.xml';sdk.write_text('fake')
        b['camera'].update(tools_dir=str(camera_dir),healthcheck_sha256=task.digest(runtime),sdk_config=str(sdk),sdk_config_sha256=task.digest(sdk))
        raw=FakeRobot();backend=h.HardwareBackend(self.j);lock=Mock();fcntl=types.SimpleNamespace(flock=Mock(),LOCK_EX=1,LOCK_NB=2)
        def configure(interface,*args):
            if args[0]=='type':raise OSError('Injected CAN initialization failure')
        with patch.object(h.sys,'platform','linux'),patch.object(h,'ObservationDashboard',return_value=raw),patch.object(h.signal,'SIGHUP',1,create=True),patch.object(h.signal,'getsignal',return_value=signal.SIG_DFL),patch.object(h.signal,'signal'),patch.dict(sys.modules,{'fcntl':fcntl}),patch('builtins.open',return_value=lock),patch.object(h.grip.can,'validate_usb_interface',return_value={'driver':'peak_usb'}),patch.object(h.grip.can,'ip_snapshot',return_value={'flags':[]}),patch.object(h.grip.can,'configure',side_effect=configure) as config:
            result=task.TaskRunner(b,backend,self.j).run()
        self.assertEqual(result['status'],'aborted');self.assertEqual(raw.commands.count('Stop()'),1);self.assertTrue(raw.closed);lock.close.assert_called_once()
        self.assertIn(unittest.mock.call('can1','down'),config.call_args_list)
    def test_gripper_failure_stops_before_saving_failed_stage(self):
        raw=FakeRobot();backend=h.HardwareBackend(self.j);backend.robot=h.RecordingRobot(raw,self.j);backend.raw_robot=raw
        backend.expected_q=[-66.5019,-74.9236,42.5551,-57.6311,90,9.1974];backend.channel=Mock()
        self.j.artifact=Mock(side_effect=OSError('Disk full'))
        with patch.object(h.grip,'run_stage',side_effect=TimeoutError('No gripper reply')),self.assertRaises(OSError):backend.gripper_stage('open')
        self.assertEqual(raw.commands.count('Stop()'),1)
    def test_stale_or_failed_monitor_prevents_motion(self):
        from tools.grip_monitor import HoldingMonitor
        monitor=HoldingMonitor(Mock(),103,{})
        monitor.count=20;monitor.last=0
        with self.assertRaises(Exception):monitor.guard()
        monitor.last=h.time.monotonic();monitor.error=TimeoutError('Lost CAN')
        with self.assertRaises(Exception):monitor.guard()

if __name__=='__main__':unittest.main()
