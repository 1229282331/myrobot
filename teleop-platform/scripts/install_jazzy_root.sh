#!/usr/bin/env bash
# Deliberately isolated OS dependency installation; no host APT or hardware IO.
set -euo pipefail
base=/root/e6-teleop-rebuild/jazzy-v32
phase=${1:-all}
[[ "$phase" == all || "$phase" == ros || "$phase" == verify ]]
root=$(realpath "$base/rootfs")
[[ "$root" == /root/e6-teleop-rebuild/jazzy-v32/rootfs && -f "$base/owner.json" ]]
grep -q 'VERSION_ID="24.04"' "$root/etc/os-release"
printf '%s\n' "$BASHPID" > "$base/install.pid"
mount -t proc proc "$root/proc" -o nosuid,nodev,noexec
trap 'umount "$root/proc"; printf "%s\n" "installer finished"' EXIT
run() { chroot "$root" /usr/bin/env DEBIAN_FRONTEND=noninteractive LC_ALL=C.UTF-8 "$@"; }
if [[ "$phase" == all ]]; then
run dpkg-query -W > "$base/dpkg-before.txt"
# Initial base image may lack CA certificates, but the Ubuntu HTTP archive has
# signed metadata. Bootstrap CA before querying the signed ROS HTTPS archive.
mv "$root/etc/apt/sources.list.d/ros2.list" "$base/ros2.list.pending"
run apt-get update
run apt-get install -y ca-certificates curl gnupg locales python3 python3-venv cmake build-essential
mv "$base/ros2.list.pending" "$root/etc/apt/sources.list.d/ros2.list"
fi
if [[ "$phase" != verify ]]; then
run apt-get update
run apt-cache policy ros-jazzy-moveit-servo | tee "$base/servo-policy.txt"
version=2.12.4-1noble.20261006.043301
candidate=$(run apt-cache policy ros-jazzy-moveit-servo | awk '/Candidate:/ {print $2}')
[[ "$candidate" == "$version" ]] || { printf 'Unreviewed Servo release: %s\n' "$candidate"; exit 2; }
printf '%s\n' "$version" > "$base/servo-version.txt"
run apt-get -s install "ros-jazzy-moveit-servo=$version" ros-jazzy-rclpy ros-jazzy-kdl-parser python3-scipy python3-yaml > "$base/install-plan.txt"
run apt-get install -y "ros-jazzy-moveit-servo=$version" ros-jazzy-rclpy ros-jazzy-kdl-parser python3-scipy python3-yaml
fi
run dpkg-query -W > "$base/dpkg-after.txt"
run /bin/bash -c 'source /opt/ros/jazzy/setup.bash; python3 -c "import rclpy; from ament_index_python.packages import get_package_prefix; print(rclpy.__file__); print(get_package_prefix(\"moveit_servo\"))"' > "$base/runtime-check.txt"
date -u +%FT%TZ > "$base/installed.txt"
