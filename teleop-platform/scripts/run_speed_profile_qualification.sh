#!/usr/bin/env bash
set -euo pipefail
base=/root/e6-teleop-rebuild/jazzy-v32
root=$(realpath "$base/rootfs")
[[ "$root" == /root/e6-teleop-rebuild/jazzy-v32/rootfs && -f "$base/owner.json" ]]
name=${1:?result name}; profile=${2:?profile}; cases=${3:?cases}
shift 3
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]]
[[ "$profile" == commissioning || "$profile" == intermediate || "$profile" == normal ]]
[[ "$cases" == joint || "$cases" == cartesian || "$cases" == all || "$cases" == singularity ]]
source_path=/qualification/speed-v41/$name
[[ ! -e "$base/speed-v41/$name" ]]
# Share the same ownership lock with the production launcher. This test opens
# no E6/CAN sockets, but it must not compete with a live ROS control runtime.
exec 9>/run/e6-phosphobot-session.lock
flock -n 9
if mountpoint -q "$root/proc"; then echo 'Previous runtime still mounted' >&2; exit 3; fi
mount -t proc proc "$root/proc" -o nosuid,nodev,noexec
trap 'umount "$root/proc"' EXIT
mkdir -p "$base/speed-v41"
script=qualify_speed_profiles.py
args=(--cases "$cases")
if [[ "$cases" == singularity ]]; then script=qualify_singularity_recovery.py; args=(); fi
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
E6_JAZZY_PYTHON_PATH="$source_path/python" \
PYTHONPATH="$root$source_path/python:/root/e6-teleop-rebuild/gateway-deps" \
python3 "$root$source_path/$script" --bundle "$root/qualification" \
  --output "$base/speed-v41/$name" --motion-profile "$profile" "${args[@]}" "$@"
