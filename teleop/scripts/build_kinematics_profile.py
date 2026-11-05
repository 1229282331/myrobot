"""Build a verified production calibration bundle from read-only raw evidence.

This changes no historical experiment/profile, and cannot unlock device IO.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.kinematics_profile import load_profile,source_sha
from e6_observer.protocol import SDK_COMMIT


def build(capture,directory):
    directory.mkdir(parents=True,exist_ok=False)
    capture=Path(capture);target=directory/capture.name
    shutil.copyfile(capture,target)
    p=dict(schema_version=1,dataset_kind='controller_kinematics_calibration',robot_ip='192.168.1.4',user=0,tool=0,
           layout_source_commit=SDK_COMMIT,source_sha256=source_sha(),capture_file=target.name,
           capture_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),physical_accuracy_verified=False)
    profile=directory/'profile.json';profile.write_text(json.dumps(p,indent=2),encoding='utf-8')
    _,metadata=load_profile(profile)
    (directory/'verification.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();build(a.capture,a.output)
