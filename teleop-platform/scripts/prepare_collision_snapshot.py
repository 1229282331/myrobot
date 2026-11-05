"""Add request-state collision evaluation to the pinned retreat Servo sources.

The default background API retains its policy. The E6 adapter opts into an
explicit actual-state input; there is no API accepting a caller-supplied scale.
This preparation does not deploy anything or change collision parameters.
"""
from pathlib import Path
import argparse
import difflib
import hashlib
import json


def prepare(source: Path):
    expected = {
        "src/collision_monitor.cpp": "92ec7b1be146ee96d9e569258f43232341424ae03d2e1a7ec87973db884c3168",
        "src/servo.cpp": "6081bd719e12a49b9a7578b1e6b4c73a67f8833e01ebd5e2a3aed8b565ec2ae3",
        "include/moveit_servo/collision_monitor.hpp": "f9054d265fdb2868ba328ffa747e25fec18f96dec097729cf3f9c6051e68b00e",
        "include/moveit_servo/servo.hpp": "ec167639e10caa6a0109eb72ebe246967cf872e366d0c5fb9b09f4a18ae00a13",
    }
    for name, digest in expected.items():
        if hashlib.sha256((source / name).read_bytes()).hexdigest() != digest:
            raise ValueError("Unexpected pinned retreat source: " + name)
    patches = []

    def edit(name, transform):
        path = source / name
        before = path.read_text(encoding="utf8")
        after = transform(before)
        assert before != after, name
        path.write_text(after, encoding="utf8", newline="\n")
        patches.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
            fromfile="retreat/" + name, tofile="snapshot/" + name))

    def replace(text, old, new):
        assert text.count(old) == 1, old
        return text.replace(old, new)

    def monitor_header(text):
        text = replace(text, "#include <moveit_servo/moveit_servo_lib_parameters.hpp>",
            "#include <moveit_servo/moveit_servo_lib_parameters.hpp>\n#include <moveit_servo/arm_proximity.hpp>\n#include <sstream>\n#include <map>")
        text = replace(text, "class CollisionMonitor", """struct CollisionSample
{
  double scale = 1.0;
  double self_distance = 0.0, scene_distance = 0.0;
  bool checked = false, self_collision = false, scene_collision = false;
  // Exact in-range distances only, associated with the checked state and ACM.
  moveit::core::RobotModelConstPtr model;
  collision_detection::CollisionEnvConstPtr environment;
  std::vector<double> positions;
  std::string matrix;
  std::map<std::pair<std::string,std::string>,double> exact_self_distances;
};

class CollisionMonitor""")
        text = replace(text, "  void stop();", """  void stop();

  // Evaluate this exact state under one scene read lock. All per-call results
  // are local, so background and request-state callers never share scratch.
  CollisionSample evaluate(const moveit::core::RobotState& actual) const;""")
        text = replace(text, "private:\n", """private:
  CollisionSample evaluateLocked(const planning_scene::PlanningScene& scene,
      const moveit::core::RobotState& actual) const;
""")
        text = replace(text, "  moveit::core::RobotState robot_state_;", "  const ArmProximityPolicy proximity_policy_;")
        text = replace(text, "  std::atomic<bool> stop_requested_;", "  std::atomic<bool> stop_requested_{true};")
        start = text.index("  // The data structures used to get information about robot self collisions.")
        end = text.index("};", start)
        return text[:start] + text[end:]

    def monitor_cpp(text):
        text = replace(text, "  , robot_state_(planning_scene_monitor->getPlanningScene()->getCurrentState())",
            "  , proximity_policy_(planning_scene_monitor->getRobotModel(), servo_params.move_group_name)")
        start = text.index("  scene_collision_request_.distance = true;")
        end = text.index("\n}", start)
        text = text[:start] + text[end:]
        start = text.index("void CollisionMonitor::checkCollisions()")
        return text[:start] + r'''CollisionSample CollisionMonitor::evaluate(const moveit::core::RobotState& actual) const
{
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
  return evaluateLocked(*scene.operator->(), actual);
}

CollisionSample CollisionMonitor::evaluateLocked(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& actual) const
{
  if (actual.getRobotModel().get() != planning_scene_monitor_->getRobotModel().get())
    throw std::invalid_argument("Collision feedback model mismatch");
  for (size_t i = 0; i < actual.getRobotModel()->getVariableCount(); ++i)
    if (!std::isfinite(actual.getVariablePositions()[i]))
      throw std::invalid_argument("Nonfinite collision feedback");
  CollisionSample sample;
  if (!servo_params_.check_collisions)
    return sample;

  moveit::core::RobotState state(actual);
  state.updateCollisionBodyTransforms();
  collision_detection::CollisionRequest request;
  request.distance = true;
  request.group_name = servo_params_.move_group_name;
  collision_detection::CollisionResult self, world;
  scene.getCollisionEnv()->checkRobotCollision(request, world, state, scene.getAllowedCollisionMatrix());
  // Original ACM for actual collisions; invariant pairs only affect distance.
  auto collision_request = request;
  collision_request.distance = false;
  const auto& environment = scene.getCollisionEnvUnpadded();
  environment->checkSelfCollision(collision_request, self, state, scene.getAllowedCollisionMatrix());
  if (self.collision)
    self.distance = 0.0;
  else
  {
    auto distance_acm = proximity_policy_.distanceMatrix(scene.getAllowedCollisionMatrix());
    collision_detection::DistanceRequest distance_request;
    distance_request.group_name = request.group_name;
    distance_request.enableGroup(state.getRobotModel());
    distance_request.acm = &distance_acm;
    // FCL's documented bounded query still computes every distance that can
    // change the original scale. At/above the unchanged threshold scale is 1.
    // SINGLE honors distance_threshold; GLOBAL replaces it with its current
    // minimum in the pinned MoveIt callback, so cannot be used for this bound.
    distance_request.type = collision_detection::DistanceRequestType::SINGLE;
    distance_request.distance_threshold = servo_params_.self_collision_proximity_threshold;
    collision_detection::DistanceResult distance_result;
    environment->distanceSelf(distance_request, distance_result, state);
    for (const auto& item : distance_result.distances)
    {
      double minimum = std::numeric_limits<double>::infinity();
      for (const auto& entry : item.second)
        minimum = std::min(minimum, entry.distance);
      if (std::isfinite(minimum) && minimum > 0.0 && minimum < distance_request.distance_threshold)
        sample.exact_self_distances[std::minmax(item.first.first,item.first.second)] = minimum;
    }
    const double distance = distance_result.minimum_distance.distance;
    // A missing in-range distance is unknown beyond the query threshold,
    // not a measured far-away clearance. The adapter emits it as JSON null.
    self.distance = distance < distance_request.distance_threshold ?
        distance : std::numeric_limits<double>::infinity();
  }
  sample.checked = true;
  sample.model = actual.getRobotModel();
  sample.environment = environment;
  sample.positions.assign(actual.getVariablePositions(), actual.getVariablePositions()+sample.model->getVariableCount());
  std::ostringstream matrix;
  scene.getAllowedCollisionMatrix().print(matrix);
  sample.matrix = matrix.str();
  sample.self_distance = self.distance;
  sample.scene_distance = world.distance;
  sample.self_collision = self.collision;
  sample.scene_collision = world.collision;
  if (self.collision || world.collision)
    sample.scale = 0.0;
  else
  {
    const double log_val = -log(0.001);
    const double self_coefficient = log_val / servo_params_.self_collision_proximity_threshold;
    const double scene_coefficient = log_val / servo_params_.scene_collision_proximity_threshold;
    const double self_scale = self.distance < servo_params_.self_collision_proximity_threshold ?
        std::exp(self_coefficient * (self.distance - servo_params_.self_collision_proximity_threshold)) : 1.0;
    const double scene_scale = world.distance < servo_params_.scene_collision_proximity_threshold ?
        std::exp(scene_coefficient * (world.distance - servo_params_.scene_collision_proximity_threshold)) : 1.0;
    sample.scale = std::min(scene_scale, self_scale);
  }
  return sample;
}

void CollisionMonitor::checkCollisions()
{
  rclcpp::WallRate rate(servo_params_.collision_check_rate);
  while (rclcpp::ok() && !stop_requested_)
  {
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
      // Keep state, world, and ACM under the same read lock, as in the original.
      collision_velocity_scale_ = evaluateLocked(*scene.operator->(), scene->getCurrentState()).scale;
    }
    rate.sleep();
  }
}
}  // namespace moveit_servo
'''

    def servo_header(text):
        text = replace(text, "class Servo\n", "enum class CollisionCheckMode { BACKGROUND, REQUEST_STATE };\n\nclass Servo\n")
        text = replace(text, "        const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);",
            "        const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,\n        CollisionCheckMode collision_mode = CollisionCheckMode::BACKGROUND);")
        text = replace(text, "  KinematicState getNextJointState(const moveit::core::RobotStatePtr& robot_state, const ServoInput& command);",
            """  KinematicState getNextJointState(const moveit::core::RobotStatePtr& robot_state, const ServoInput& command,
      const moveit::core::RobotStateConstPtr& actual_collision_state = nullptr);
  const CollisionSample& getCollisionSample() const { return collision_sample_; }""")
        text = replace(text, "private:\n", """private:
  const CollisionCheckMode collision_mode_;
  CollisionSample collision_sample_;
""")
        return text

    def servo_cpp(text):
        text = replace(text, "             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)\n  : node_(node)",
            "             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,\n             CollisionCheckMode collision_mode)\n  : collision_mode_(collision_mode)\n  , node_(node)")
        text = replace(text, "  collision_monitor_->start();", "  if (collision_mode_ == CollisionCheckMode::BACKGROUND)\n    collision_monitor_->start();")
        text = replace(text, "  check_collision ? collision_monitor_->start() : collision_monitor_->stop();",
            "  (check_collision && collision_mode_ == CollisionCheckMode::BACKGROUND) ?\n      collision_monitor_->start() : collision_monitor_->stop();")
        text = replace(text, "KinematicState Servo::getNextJointState(const moveit::core::RobotStatePtr& robot_state, const ServoInput& command)\n{",
            """KinematicState Servo::getNextJointState(const moveit::core::RobotStatePtr& robot_state, const ServoInput& command,
    const moveit::core::RobotStateConstPtr& actual_collision_state)
{
  if ((collision_mode_ == CollisionCheckMode::REQUEST_STATE) != bool(actual_collision_state))
    throw std::invalid_argument("Collision mode requires its matching actual-state input");""")
        text = replace(text, "  double collision_scale = collision_velocity_scale_.load();", """  collision_sample_ = collision_mode_ == CollisionCheckMode::REQUEST_STATE ?
      collision_monitor_->evaluate(*actual_collision_state) : CollisionSample{};
  const double collision_scale = collision_mode_ == CollisionCheckMode::REQUEST_STATE ?
      collision_sample_.scale : collision_velocity_scale_.load();""")
        return text

    edit("include/moveit_servo/collision_monitor.hpp", monitor_header)
    edit("src/collision_monitor.cpp", monitor_cpp)
    edit("include/moveit_servo/servo.hpp", servo_header)
    edit("src/servo.cpp", servo_cpp)
    (source / "collision-snapshot.patch").write_text("".join(patches), encoding="utf8", newline="\n")
    info = {name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in (
        "include/moveit_servo/collision_monitor.hpp", "src/collision_monitor.cpp",
        "include/moveit_servo/servo.hpp", "src/servo.cpp")}
    (source / "collision-snapshot-source.json").write_text(json.dumps(info, indent=2))
    return info


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source)))
