"""Load the same immutable command profile deployed from the PC."""
import json
from pathlib import Path


def gripper_profile(name):
    profiles=json.loads((Path(__file__).resolve().parents[2]/'config/gripper_profiles.json').read_text(encoding='utf-8'))
    if name not in profiles:raise ValueError('Unknown gripper command profile')
    return dict(profiles[name])


def select_command(candidate,previous,minimum_counts):
    """Accumulate small requested increments without replacing the sent goal.

    The device can report state=0 far from its absolute goal after small
    updates. Keep exchanging the last accepted byte target until the new
    target changes by the tested number of counts. No overshoot, retry,
    parameter adjustment or delayed execution after release is introduced.
    """
    if type(minimum_counts) is not int or not 1<=minimum_counts<=4:
        raise ValueError('Unsupported gripper target resolution')
    if len(candidate)!=8 or previous is not None and len(previous)!=8:
        raise ValueError('Expected an eight-byte classical CAN command')
    if previous is not None:
        if candidate[:1]+candidate[2:]!=previous[:1]+previous[2:]:
            raise ValueError('Gripper command parameters changed during control')
        if abs(candidate[1]-previous[1])<minimum_counts:return previous
    return candidate
