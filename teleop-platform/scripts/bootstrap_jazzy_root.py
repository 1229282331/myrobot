"""Prepare a pinned Ubuntu ARM64 root for isolated ROS Jazzy qualification.

Run on X5 as root. No robot connections, host APT changes, drivers, Docker
daemon, or device mounts. ROS is installed by install_jazzy_root.sh afterward.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import urllib.request

BASE=Path('/root/e6-teleop-rebuild/jazzy-v32')
IMAGE='ubuntu-base-24.04.4-base-arm64.tar.gz'
SHA='04207713ece899c3740823d33690441ad3a7f0ded1101aca744e2b0f37ac7ff2'
URL='https://cdimage.ubuntu.com/ubuntu-base/releases/noble/release/'+IMAGE


def download(url,path):
    with urllib.request.urlopen(url,timeout=45) as response,path.open('wb') as out:
        shutil.copyfileobj(response,out)


def main():
    if os.geteuid()!=0 or os.uname().machine!='aarch64':raise RuntimeError('X5 ARM64 root required')
    BASE.mkdir(parents=True,exist_ok=True);root=BASE/'rootfs'
    if root.exists():raise RuntimeError('Root already exists; inspect the previous run, never overwrite it')
    archive=BASE/IMAGE
    if not archive.exists():download(URL,archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=SHA:raise RuntimeError('Ubuntu image hash mismatch')
    root.mkdir()
    subprocess.run(['tar','-xpf',str(archive),'-C',str(root)],check=True)
    for name,major,minor in [('null',1,3),('zero',1,5),('random',1,8),('urandom',1,9)]:
        target=root/'dev'/name
        if not target.exists():subprocess.run(['mknod','-m','666',str(target),'c',str(major),str(minor)],check=True)
    dns=root/'etc/resolv.conf';dns.unlink(missing_ok=True);shutil.copyfile('/etc/resolv.conf',dns)
    (root/'usr/sbin/policy-rc.d').write_text('#!/bin/sh\nexit 101\n');(root/'usr/sbin/policy-rc.d').chmod(0o755)
    (root/'etc/apt/apt.conf.d/99teleop-root').write_text('APT::Install-Recommends "false";\nAPT::Install-Suggests "false";\nAcquire::Retries "3";\n')
    key=BASE/'ros.key';download('https://raw.githubusercontent.com/ros/rosdistro/master/ros.key',key)
    shutil.copyfile(key,root/'usr/share/keyrings/ros-archive-keyring.gpg')
    # ROS's public APT archive uses HTTP with signed Release files/packages;
    # HTTPS on this host is not a supported certificate endpoint.
    (root/'etc/apt/sources.list.d/ros2.list').write_text('deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main\n')
    metadata=dict(kind='isolated_jazzy_root',root=str(root),ubuntu_url=URL,ubuntu_sha256=SHA,
        ros_key_sha256=hashlib.sha256(key.read_bytes()).hexdigest(),host_packages_changed=False,
        hardware_io=False,services_suppressed=True)
    (BASE/'owner.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata))


if __name__=='__main__':main()
