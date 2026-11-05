#!/usr/bin/env bash
# X5 build halves. Compile the large unit on PC with GCC 11 targeting ARM64.
# Does not connect to E6, initialize CAN, or launch a ROS robot driver.
set -euo pipefail
bundle=$(realpath "${1:?usage: build_moveit_bundle.sh BUNDLE prepare|finish [CROSS_OBJECT]}")
mode=${2:?prepare or finish}
source /opt/ros/humble/setup.bash
cd "$bundle/vendor/moveit_servo"
if [[ "$mode" == prepare ]]; then
  patch --batch -p1 < "$bundle/provenance.patch"
  cmake -S . -B "$bundle/servo-build" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O1 -DNDEBUG"
  cmake --build "$bundle/servo-build" --target src/servo_calcs.cpp.i -j1
  unit="$bundle/servo-build/CMakeFiles/moveit_servo_lib.dir/src/servo_calcs.cpp.i"
  gzip -c "$unit" > "$bundle/servo_calcs.cpp.i.gz"
  sha256sum "$unit" > "$bundle/preprocessed-sha256.txt"
  printf '%s\n' 'Copy this preprocessed unit to PC, run compile_moveit_unit.py, then finish with its object and .json sidecar.'
  exit 0
fi
[[ "$mode" == finish ]] || { printf '%s\n' 'Unknown mode' >&2; exit 2; }
object=$(realpath "${3:?cross-compiled object with .json provenance sidecar required}")
python3 - "$bundle" "$object" <<'PY'
from pathlib import Path
import hashlib,json,shutil,sys
bundle,obj=map(Path,sys.argv[1:]);metadata=json.loads(Path(str(obj)+'.json').read_text())
target=bundle/'servo-build/CMakeFiles/moveit_servo_lib.dir/src/servo_calcs.cpp.o'
unit=target.with_suffix('.i')
assert hashlib.sha256(unit.read_bytes()).hexdigest()==metadata['preprocessed_sha256'], 'Different native headers/source'
assert hashlib.sha256(obj.read_bytes()).hexdigest()==metadata['object_sha256'], 'Object changed after compilation'
shutil.copyfile(obj,target)
PY
# Limit remaining small build processes only. The checked core .o is new.
ulimit -v 2300000
cmake --build "$bundle/servo-build" --target servo_node_main -j1
g++ -std=c++17 -O2 "$bundle/kdl_model_probe.cpp" \
  -I/opt/ros/humble/include/kdl_parser -I/opt/ros/humble/include/urdf \
  -I/opt/ros/humble/include/urdf_parser_plugin -I/opt/ros/humble/include/rcutils \
  -I/opt/ros/humble/include/rclcpp -I/opt/ros/humble/include/urdfdom_headers \
  -I/usr/include/eigen3 -L/opt/ros/humble/lib -lkdl_parser -lorocos-kdl \
  -o "$bundle/kdl_model_probe"
sha256sum "$bundle/servo-build/servo_node_main" "$bundle/servo-build/libmoveit_servo_lib.so" \
  "$bundle/servo-build/libservo_node.so" "$bundle/servo-build/libmoveit_servo_lib_parameters.so" \
  "$bundle/kdl_model_probe" > "$bundle/binary-sha256.txt"
