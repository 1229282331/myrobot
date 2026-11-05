"""Fault/lifecycle qualification of the formal C++ bridge, virtual plant only."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
from e6_phosphobot.intent import Intent
from e6_phosphobot.moveit_bridge import MoveItBridge
from qualify_jazzy_bridge import Plant


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    plant=Plant(a.bundle,.15);bridge=MoveItBridge(a.bundle,plant,motion_profile='normal',api='cpp')
    result=dict(hardware_io=False,events=[],error=None);paused=None
    intent=Intent(linear_m_s=[0.]*3,angular_rad_s=[0.]*3,joint_rad_s=[0.]*5+[math.radians(1)],
                  frame='base',deadman=True,gripper_rate_s=0.)
    def event(name,**extra):result['events'].append(dict(name=name,at_ns=time.perf_counter_ns(),**extra))
    def wait(predicate,timeout=3):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            plant.update()
            if predicate():return
            time.sleep(.005)
        raise TimeoutError(str(bridge.snapshot()))
    def owned_cpp_pid():
        pid=bridge.snapshot()['servo_pid']
        fields=Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()
        assert int(fields[1])==bridge.process.pid
        assert b'servo_cpp_adapter' in Path('/proc',str(pid),'cmdline').read_bytes()
        return pid
    def advance(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            plant.update();time.sleep(.005)
    try:
        bridge.start();wait(lambda:bridge.snapshot()['ready'],40)
        for seq in range(1,11):
            bridge.submit(intent,seq)
            wait(lambda:bridge.snapshot().get('first_candidate') is not None)
            row=bridge.take();assert row is not None and row['input_sequence']==seq
            bridge.stop();plant.stop_fixture()
            assert bridge.take() is None
            wait(lambda:bridge.snapshot()['phase']=='paused')
        event('ten_start_release_cycles',passed=True)
        bridge.submit(intent,11);wait(lambda:bridge.snapshot().get('first_candidate') is not None)
        advance(.28)
        wait(lambda:bridge.snapshot()['phase']=='paused')
        assert bridge.take() is None
        event('input_expiry',passed=True,info=bridge.snapshot())
        bridge.stop()
        # Capture only this bridge's exact C++ child. A stalled calculation
        # cannot produce fresh endpoints, and its late output is discarded.
        paused=owned_cpp_pid();os.kill(paused,signal.SIGSTOP)
        bridge.submit(intent,12)
        advance(.18);assert bridge.take() is None
        bridge.stop();os.kill(paused,signal.SIGCONT);paused=None
        wait(lambda:bridge.snapshot()['phase']=='paused')
        assert bridge.take() is None
        event('stalled_cpp_and_release',passed=True,info=bridge.snapshot())
        bridge.submit(intent,13);wait(lambda:bridge.snapshot().get('first_candidate') is not None)
        assert bridge.take() is not None
        bridge.stop();wait(lambda:bridge.snapshot()['phase']=='paused')
        event('fresh_epoch_after_stall',passed=True)
        # A solver crash must not stop the independent feedback provider.
        os.kill(owned_cpp_pid(),signal.SIGKILL)
        wait(lambda:bridge.snapshot()['phase']=='failed',8)
        assert bridge.take() is None
        plant.update();assert plant.snapshot()['connected']
        event('cpp_exit_feedback_provider_survives',passed=True,info=bridge.snapshot())
        result['passed']=True
    except Exception as exc:result['error']=str(exc)
    finally:
        if paused is not None:
            try:os.kill(paused,signal.SIGCONT)
            except ProcessLookupError:pass
        bridge.close();plant.close()
        result['bridge']=bridge.snapshot();result['child_exit']=bridge.process.returncode if bridge.process else None
        (a.output/'summary.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result))
    return int(result['error'] is not None or not result.get('passed'))


if __name__=='__main__':raise SystemExit(main())
