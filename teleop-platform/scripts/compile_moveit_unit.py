"""Compile X5-preprocessed Servo core on PC Linux, preserving native headers."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    compiler='aarch64-linux-gnu-g++-11'
    version=subprocess.check_output([compiler,'--version'],text=True).splitlines()[0]
    command=[compiler,'-x','c++-cpp-output','-O1','-DNDEBUG','-fPIC','-std=c++17',
             '-c',str(a.input),'-o',str(a.output)]
    subprocess.run(command,check=True)
    Path(str(a.output)+'.json').write_text(json.dumps(dict(compiler=version,command=command,
        preprocessed_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),
        object_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest()),indent=2))
