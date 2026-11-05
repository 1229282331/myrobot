#!/usr/bin/env bash
set -euo pipefail
base=/root/e6-teleop-rebuild/jazzy-v32
root=$(realpath "$base/rootfs")
[[ "$root" == /root/e6-teleop-rebuild/jazzy-v32/rootfs && -f "$base/owner.json" ]]
name=${1:?result name}; suite=${2:-speed}; profile=${3:-commissioning}
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ && ( "$suite" == speed || "$suite" == directions || "$suite" == lifecycle ) ]]
[[ "$profile" == commissioning || "$profile" == intermediate || "$profile" == normal ]]
[[ ! -e "$base/bridge-v36/$name" ]]
if mountpoint -q "$root/proc"; then echo 'Previous owned runtime still mounted' >&2; exit 3; fi
mount -t proc proc "$root/proc" -o nosuid,nodev,noexec
trap 'umount "$root/proc"' EXIT
mkdir -p "$base/bridge-v36"
if [[ "$suite" == lifecycle ]]; then
 OPENBLAS_NUM_THREADS=1 PYTHONPATH="$root/qualification/python-v36:/root/e6-teleop-rebuild/gateway-deps" \
 python3 "$root/qualification/qualify_jazzy_lifecycle.py" --bundle "$root/qualification" --output "$base/bridge-v36/$name"
else
 OPENBLAS_NUM_THREADS=1 PYTHONPATH="$root/qualification/python-v36:/root/e6-teleop-rebuild/gateway-deps" \
 python3 "$root/qualification/qualify_jazzy_bridge.py" --bundle "$root/qualification" \
 --output "$base/bridge-v36/$name" --suite "$suite" --motion-profile "$profile"
fi
