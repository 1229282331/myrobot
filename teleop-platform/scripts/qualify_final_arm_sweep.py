"""Compile and test final-target collision coverage using original MoveIt/FCL.

Linux with ROS Jazzy/MoveIt development packages is required. This program has
no E6 SDK or actuation interface. Supply the same model parameters as the native
adapter, with mesh paths resolved for the machine running the check.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from urllib.parse import unquote, urlparse

import yaml


def geometry_binding(params_file):
    """Bind geometry contents while allowing machine-local mesh locations."""
    document = yaml.safe_load(params_file.read_text())
    assert len(document) == 1, "Provide one wildcard ROS parameter block"
    params = next(iter(document.values()))["ros__parameters"]
    meshes = {}

    def mesh(match):
        uri = match.group(1)
        parsed = urlparse(uri)
        assert parsed.scheme in ("", "file") and not parsed.netloc, uri
        path = Path(unquote(parsed.path))
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        meshes[uri] = digest
        return 'filename="sha256:' + digest + '"'

    geometry = {k: v for k, v in params.items()
                if k.startswith("e6_gripper_") or k == "robot_description_semantic"}
    geometry["robot_description"] = re.sub(
        r'filename="([^"]+)"', mesh, params["robot_description"])
    geometry["arm_group"] = params["moveit_servo"]["move_group_name"]
    digest = hashlib.sha256(json.dumps(geometry, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return dict(geometry_sha256=digest, mesh_sha256=meshes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    fixture = json.loads((root / "tests/fixtures/final_arm_sweep_v348.json").read_text())
    binding = geometry_binding(args.params)
    assert binding["geometry_sha256"] == fixture["model_binding"]["geometry_sha256"], "Wrong geometry or ACM"
    args.output.mkdir(parents=True, exist_ok=False)
    source = args.output / "source"
    source.mkdir()
    for name in ("dynamic_gripper.hpp", "sweep_distance.hpp"):
        shutil.copy2(args.source_dir / name, source / name)
    shutil.copy2(root / "scripts/probe_final_arm_sweep.cpp", source / "probe.cpp")
    (source / "CMakeLists.txt").write_text("""cmake_minimum_required(VERSION 3.22)
project(e6_final_arm_sweep_probe)
find_package(ament_cmake REQUIRED)
foreach(dep moveit_core moveit_ros_planning rclcpp rclcpp_lifecycle)
  find_package(${dep} REQUIRED)
endforeach()
add_executable(probe probe.cpp)
target_include_directories(probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(probe moveit_core moveit_ros_planning rclcpp rclcpp_lifecycle)
target_compile_features(probe PRIVATE cxx_std_17)
""")
    cases = fixture["cases"]
    inputs = "\n".join(" ".join(map(str, [*x["actual_rad"], *x["target_rad"], x["raw"]]))
                       for x in cases) + "\n"
    (args.output / "inputs.txt").write_text(inputs)
    (args.output / "model-binding.json").write_text(json.dumps(binding, indent=2))
    (args.output / "source-manifest.json").write_text(json.dumps({
        f.name: hashlib.sha256(f.read_bytes()).hexdigest()
        for f in source.iterdir() if f.is_file()}, indent=2))
    env = dict(os.environ, ROS_LOCALHOST_ONLY="1", OPENBLAS_NUM_THREADS="1")
    phase = "build"
    result = dict(passed=False, physical_io=False)
    try:
        with (args.output / "build.log").open("w") as log:
            subprocess.run(["cmake", "-S", str(source), "-B", str(args.output / "build"),
                            "-DCMAKE_BUILD_TYPE=Release"],
                           stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
            subprocess.run(["cmake", "--build", str(args.output / "build"), "--parallel", "2"],
                           stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
        phase = "probe"
        with (args.output / "stderr.log").open("w") as err:
            run = subprocess.run([str(args.output / "build/probe"), "--ros-args",
                                  "--disable-external-lib-logs", "--params-file", str(args.params)],
                                 input=inputs, stdout=subprocess.PIPE, stderr=err,
                                 text=True, env=env, timeout=120)
        (args.output / "stdout.jsonl").write_text(run.stdout)
        observations = [json.loads(line) for line in run.stdout.splitlines()]
        assert run.returncode == 0 and len(observations) == len(cases)
        for case, observed in zip(cases, observations):
            assert not observed["budget_exhausted"], case["name"]
            assert observed["target_collision"] == case["target_collision"], case["name"]
            assert observed["target_contacts"] == case["target_contacts"], case["name"]
            assert observed["arm_safe"] == case["expected_arm_safe"], case["name"]
            assert not (observed["arm_safe"] and observed["colliding_samples"]), case["name"]
        result.update(passed=True, cases=len(cases), dense_checks=65*len(cases))
    except Exception as exc:
        result.update(error=repr(exc))
        raise
    finally:
        result["phase"] = phase
        (args.output / "result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result))


if __name__ == "__main__":
    main()
