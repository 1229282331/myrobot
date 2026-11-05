#!/usr/bin/env bash
# Run a virtual-only upstream Servo qualification in the owned Ubuntu root.
set -euo pipefail
base=/root/e6-teleop-rebuild/jazzy-v32
root=$(realpath "$base/rootfs")
[[ "$root" == /root/e6-teleop-rebuild/jazzy-v32/rootfs && -f "$base/owner.json" ]]
[[ -f "$base/installed.txt" ]]
name=${1:?result name}; tau=${2:?plant lag seconds}
plugin=${3:-AccelerationLimitedPlugin}
speed=${4:-0.2}
period=${5:-0.05}
node_kind=${6:-original}
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ && "$tau" =~ ^[0-9.]+$ ]]
[[ "$speed" =~ ^-?[0-9.]+$ ]]
[[ "$period" == 0.05 || "$period" == 0.02 || "$period" == 0.01 ]]
[[ "$node_kind" == original || "$node_kind" == traced ]]
[[ "$plugin" == AccelerationLimitedPlugin || "$plugin" == RuckigFilterPlugin || "$plugin" == ReviewedAccelerationLimitedPlugin ]]
[[ ! -e "$root/qualification/results/$name" ]]
# Never unmount another installer's or runtime's mount.
if mountpoint -q "$root/proc"; then
    printf 'Owned root is still mounted; inspect and release the previous run first.\n' >&2
    exit 3
fi
mount -t proc proc "$root/proc" -o nosuid,nodev,noexec
trap 'umount "$root/proc"' EXIT
mkdir -p "$root/dev/shm" "$root/tmp" "$root/qualification/results"
chmod 1777 "$root/dev/shm" "$root/tmp"
chroot "$root" /usr/bin/env ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=88 OPENBLAS_NUM_THREADS=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    /bin/bash -c 'source /opt/ros/jazzy/setup.bash; if [[ "$3" == ReviewedAccelerationLimitedPlugin ]]; then export AMENT_PREFIX_PATH="/qualification/filter-overlay-v34:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="/qualification/filter-overlay-v34/lib:$LD_LIBRARY_PATH"; fi; exec python3 /qualification/qualify_jazzy_node.py --bundle /qualification --output "/qualification/results/$1" --tau "$2" --plugin "$3" --speed-deg-s "$4" --period "$5" --node-kind "$6"' \
    bash "$name" "$tau" "$plugin" "$speed" "$period" "$node_kind"
