"""Fixed-hover scene input. Default prints configuration entirely offline.

Explicit --capture uses an already enabled hover, a single Dashboard and the
existing X5 camera runtime. It opens no CAN channel and sends no motion or
enable commands. Capture/robot failure may send one safety Stop.
"""
import argparse,json,signal,sys
from pathlib import Path
from .grasp_task import Journal,E6Error,digest,utc
from .grasp_task_plan import read_profile

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',action='store_true');p.add_argument('--output',type=Path);a=p.parse_args()
    profile=read_profile()
    if not a.capture:
        print(json.dumps(dict(status='offline_preview',required_enabled_hover=profile['reviewed_hover_q'],hardware_connections=0,camera=profile['camera']),indent=2));return 0
    if sys.platform!='linux' or a.output is None:p.error('Explicit capture requires X5/Linux and a new output directory')
    camera=profile['camera']
    if digest(Path(camera['tools_dir'])/'rdk_healthcheck.py')!=camera['healthcheck_sha256'] or digest(camera['sdk_config'])!=camera['sdk_config_sha256']:
        raise E6Error('Camera runtime/config differs from pinned installation')
    from .grasp_task_hardware import HardwareBackend,RecordingRobot,ObservationDashboard
    journal=Journal(a.output,'hardware_observation');backend=HardwareBackend(journal)
    backend.bundle=profile;backend.expected_q=profile['reviewed_hover_q'];result=dict(started_utc=utc(),status='starting',robot_motion_commands=0,can_frames=0)
    try:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            backend.previous_handlers[sig]=signal.getsignal(sig);signal.signal(sig,lambda *_:setattr(backend,'cancelled',True))
        backend.raw_robot=ObservationDashboard('192.168.1.4',timeout=.5);backend.raw_robot.__enter__();backend.robot=RecordingRobot(backend.raw_robot,journal)
        observation=backend.observe('hover')
        scene=dict(status='static_scene',before=observation['robot_before'],after=observation['robot_after'],source_run=str(Path(observation['camera_report']).parent.resolve()),camera_report_sha256=observation['camera_report_sha256'],robot_write_commands=0,ended_utc=utc())
        journal.artifact('scene_observation.json',scene);result['status']='completed'
    except BaseException as exc:
        result.update(status='aborted',error=repr(exc))
        try:result['stop']=backend.stop_once()
        except BaseException as stop_exc:result['stop_error']=repr(stop_exc)
    finally:
        try:result['cleanup']=backend.close()
        except BaseException as exc:result.update(status='cleanup_unconfirmed',cleanup_error=repr(exc))
        result['ended_utc']=utc()
        try:journal.artifact('summary.json',result)
        finally:journal.close()
    print(json.dumps({k:result.get(k) for k in ('status','robot_motion_commands','can_frames','error')},indent=2));return int(result['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
