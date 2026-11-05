"""Delivery coverage uses maximum real mapper inputs, not just cap labels."""
import json
import math
from pathlib import Path
import subprocess

def test_delivery_gamepad_exercises_full_sticks_and_triggers():
    root=Path(__file__).resolve().parents[1]
    result=subprocess.run(['node',str(root/'web/scripts/synthetic-inputs.mjs'),'','delivery'],capture_output=True,text=True,check=True)
    fixture=json.loads(result.stdout)
    assert fixture['gamepad_signal_amplitude']==1
    for row in fixture['rows']:
        if row['source']!='gamepad':continue
        signal=row['input']
        if row['axis']==3:
            assert abs(signal['gripper_rate'])==.1
        else:
            expected=.005 if row['mode']=='translation' else math.radians(2)
            assert math.isclose(math.sqrt(sum(v*v for v in signal['twist'])),expected,abs_tol=1e-12)
    for row in fixture['multi']:
        expected=.005 if row['mode']=='translation' else math.radians(2)
        assert math.isclose(math.sqrt(sum(v*v for v in row['input']['twist'])),expected,abs_tol=1e-12)
