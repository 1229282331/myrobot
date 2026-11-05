"""No-device startup and child-failure lifecycle checks for the real bridge."""
import argparse
import json
from pathlib import Path
import signal
import time
from e6_phosphobot.moveit_bridge import MoveItBridge
from qualify_jazzy_bridge import Plant


class NoFeedback:
    def snapshot(self):return dict(connected=False,age_ms=None,received_ns=None,generation=0,status=None)


def running(pid):
    path=Path('/proc',str(pid),'stat')
    return path.exists() and path.read_text().rsplit(')',1)[1].split()[0]!='Z'


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);result=dict(hardware_io=False,cases=[],error=None)
    bridge=None;plant=None
    try:
        for failure in (False,True):
            reader=NoFeedback() if not failure else Plant(a.bundle,.15)
            if failure:plant=reader
            bridge=MoveItBridge(a.bundle,reader);bridge.start();deadline=time.monotonic()+40
            while not (bridge.snapshot().get('servo_pid') and (not failure or bridge.snapshot()['ready'])):
                if plant:plant.update()
                if bridge.snapshot()['phase']=='failed' or time.monotonic()>deadline:raise RuntimeError(str(bridge.snapshot()))
                time.sleep(.005)
            before=bridge.snapshot()
            if not failure and before['ready']:raise RuntimeError('No-feedback adapter unexpectedly ready')
            if failure:
                bridge.process.send_signal(signal.SIGKILL);deadline=time.monotonic()+3
                while bridge.snapshot()['phase']!='failed':
                    plant.update()
                    if time.monotonic()>deadline:raise RuntimeError('Child failure not propagated')
                    time.sleep(.01)
            started=time.monotonic();bridge.close();elapsed=time.monotonic()-started
            time.sleep(.1)
            leftovers=[pid for pid in (before['servo_pid'],before['kdl_pid'],bridge.process.pid) if running(pid)]
            case=dict(kind='killed_child' if failure else 'no_feedback_startup',before=before,close_s=elapsed,
                      child_exit=bridge.process.returncode,leftover_running_pids=leftovers,after=bridge.snapshot())
            result['cases'].append(case)
            if leftovers or elapsed>5 or (not failure and bridge.process.returncode!=0):raise RuntimeError('Lifecycle qualification failed')
            if plant:plant.close();plant=None
            bridge=None
        result['passed']=True
    except Exception as exc:result['error']=str(exc)
    finally:
        if bridge:bridge.close()
        if plant:plant.close()
        (a.output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    return int(result['error'] is not None)


if __name__=='__main__':raise SystemExit(main())
