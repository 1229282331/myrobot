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
 * Title      : collision_monitor.hpp
 * Project    : moveit_servo
 * Created    : 06/08/2023
 * Author     : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 * Description: Monitors the planning scene for collision and publishes the velocity scaling.
 */

#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_servo/moveit_servo_lib_parameters.hpp>
#include <moveit_servo/arm_proximity.hpp>
#include <sstream>
#include <map>

class SweepDistanceCertificate;

namespace moveit_servo
{

struct ProximityWitness { Eigen::Vector3d first_local,second_local; };

struct CollisionSample
{
  double scale = 1.0;
  double self_distance = 0.0, scene_distance = 0.0;
  bool checked = false, self_collision = false, scene_collision = false;
  bool initial_distance_optimized = false;
  // Exact in-range distances only, associated with the checked state and ACM.
  moveit::core::RobotModelConstPtr model;
  collision_detection::CollisionEnvConstPtr environment;
  std::vector<double> positions;
  std::string matrix;
  std::map<std::pair<std::string,std::string>,double> exact_self_distances;
  // Optional nearest-point witnesses only choose query ordering. They never
  // establish a collision-free interval or grant a retreat speed increase.
  std::map<std::pair<std::string,std::string>,ProximityWitness> exact_self_witnesses;
};

class CollisionMonitor
{
public:
  CollisionMonitor(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                   const servo::Params& servo_params, std::atomic<double>& collision_velocity_scale);

  void start();

  void stop();

  // Evaluate this exact state under one scene read lock. All per-call results
  // are local, so background and request-state callers never share scratch.
  CollisionSample evaluate(const moveit::core::RobotState& actual) const;

private:
  CollisionSample evaluateLocked(const planning_scene::PlanningScene& scene,
      const moveit::core::RobotState& actual) const;
  /**
   * \brief The collision checking function, this will run in a separate thread.
   */
  void checkCollisions();

  // Variables

  const servo::Params& servo_params_;

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const ArmProximityPolicy proximity_policy_;
  std::shared_ptr<SweepDistanceCertificate> initial_distance_;

  // The collision monitor thread.
  std::thread monitor_thread_;
  // The flag used for stopping the collision monitor thread.
  std::atomic<bool> stop_requested_{true};

  // The scaling factor when approaching a collision.
  std::atomic<double>& collision_velocity_scale_;

};

}  // namespace moveit_servo
