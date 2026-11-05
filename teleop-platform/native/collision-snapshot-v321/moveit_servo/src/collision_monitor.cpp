/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
/*
 * Title      : collision_monitor.cpp
 * Project    : moveit_servo
 * Created    : 06/08/2023
 * Author     : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 */

#include <moveit_servo/collision_monitor.hpp>
#include <moveit_servo/arm_proximity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/utils/logger.hpp>

namespace moveit_servo
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.move_group.collision_monitor");
}
}  // namespace

CollisionMonitor::CollisionMonitor(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                   const servo::Params& servo_params, std::atomic<double>& collision_velocity_scale)
  : servo_params_(servo_params)
  , planning_scene_monitor_(planning_scene_monitor)
  , proximity_policy_(planning_scene_monitor->getRobotModel(), servo_params.move_group_name)
  , collision_velocity_scale_(collision_velocity_scale)
{

}

void CollisionMonitor::start()
{
  stop_requested_ = false;
  if (!monitor_thread_.joinable())
  {
    monitor_thread_ = std::thread(&CollisionMonitor::checkCollisions, this);
    RCLCPP_INFO_STREAM(getLogger(), "Collision monitor started");
  }
  else
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Collision monitor could not be started");
  }
}

void CollisionMonitor::stop()
{
  stop_requested_ = true;
  if (monitor_thread_.joinable())
  {
    monitor_thread_.join();
  }
  RCLCPP_INFO_STREAM(getLogger(), "Collision monitor stopped");
}

CollisionSample CollisionMonitor::evaluate(const moveit::core::RobotState& actual) const
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
