#!/usr/bin/env bash
set -euo pipefail
base=/root/e6-teleop-rebuild/jazzy-v32
root=$(realpath "$base/rootfs")
[[ "$root" == /root/e6-teleop-rebuild/jazzy-v32/rootfs && -f "$base/owner.json" ]]
name=${1:?result name}; tau=${2:?virtual tau}
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ && "$tau" =~ ^[0-9]+([.][0-9]+)?$ ]]
source_path=/qualification/reversal-v108/$name
[[ ! -e "$base/reversal-v108/$name" ]]
exec 9>/run/e6-phosphobot-session.lock
flock -n 9
if mountpoint -q "$root/proc"; then echo 'Previous runtime still mounted' >&2; exit 3; fi
mount -t proc proc "$root/proc" -o nosuid,nodev,noexec
trap 'umount "$root/proc"' EXIT
mkdir -p "$base/reversal-v108"
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
E6_JAZZY_PYTHON_PATH="$source_path/python" \
PYTHONPATH="$root$source_path/python:/root/e6-teleop-rebuild/gateway-deps" \
python3 "$root$source_path/qualify_reversal_trace.py" --bundle "$root/qualification" \
  --fixture "$root$source_path/fixture.json" --output "$base/reversal-v108/$name" --tau "$tau"
