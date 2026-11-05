"""Fixed, named CAN settings; a profile is never a runtime parameter write."""
import json
from .model import ROOT


def gripper_profile(name):
    profiles=json.loads((ROOT/'config/gripper_profiles.json').read_text(encoding='utf-8'))
    if name not in profiles:raise ValueError('Unknown gripper command profile')
    return dict(profiles[name])
