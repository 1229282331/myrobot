"""PC-side validation uses the exact lightweight X5 implementation."""
import sys
from .model import ROOT


def verify_local_profile(path):
    gateway=str(ROOT/'device_gateway')
    if gateway not in sys.path:sys.path.insert(0,gateway)
    from e6_observer.kinematics_profile import load_profile
    _,metadata=load_profile(path)
    return metadata
