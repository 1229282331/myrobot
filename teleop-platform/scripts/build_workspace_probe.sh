#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
root=$(cd "$(dirname "$0")/../.." && pwd)
build="$HOME/.cache/e6-workspace-v261"
mkdir -p "$build/source"
cp "$root/teleop-platform/scripts/workspace_probe.cpp" "$build/source/"
cp "$root/teleop-platform/scripts/workspace_probe_CMakeLists.txt" "$build/source/CMakeLists.txt"
cmake -S "$build/source" -B "$build/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build/build" -j2
dpkg-query -W ros-jazzy-moveit-core ros-jazzy-moveit-kinematics libfcl0.7 liborocos-kdl1.5
