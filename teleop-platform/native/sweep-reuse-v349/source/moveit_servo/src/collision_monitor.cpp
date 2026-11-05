#include <moveit/robot_model/aabb.hpp>
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
#include "sweep_distance.hpp"
#include <moveit/collision_detection_fcl/collision_env_fcl.hpp>
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
  initial_distance_=std::make_shared<SweepDistanceCertificate>(
      planning_scene_monitor_->getRobotModel(),servo_params_.move_group_name,true,true);
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

// Distance-only broadphase at THIS actual pose. Original collision queries
// remain unchanged. An enclosing box distance is a lower bound in every rigid
// coordinate frame; a pair proven above the exact-query threshold cannot change
// the selected proximity scale. No approximate distance enters the exact cache.
static collision_detection::AllowedCollisionMatrix boundedDistanceMatrix(
    const moveit::core::RobotState& state,
    const collision_detection::CollisionEnvConstPtr& environment,
    const collision_detection::AllowedCollisionMatrix& original, double threshold)
{
  if (!std::isfinite(threshold) || threshold <= 0.) return original;
  std::vector<const moveit::core::AttachedBody*> attached;state.getAttachedBodies(attached);
  if (!attached.empty()) return original;
  for (const auto& item : environment->getLinkScale()) if (item.second != 1.) return original;
  for (const auto& item : environment->getLinkPadding()) if (item.second != 0.) return original;
  auto result = original;
  const auto& links = state.getRobotModel()->getLinkModelsWithCollisionGeometry();
  for (size_t i=0;i<links.size();++i) for (size_t j=i+1;j<links.size();++j)
  {
    const auto* a=links[i];const auto* b=links[j];
    collision_detection::AllowedCollision::Type type;
    if (original.getAllowedCollision(a->getName(),b->getName(),type) &&
        type != collision_detection::AllowedCollision::NEVER) continue;
    for (const auto* frame : {a,b})
    {
      const auto inverse=state.getGlobalLinkTransform(frame).inverse();
      auto ta=inverse*state.getGlobalLinkTransform(a);
      auto tb=inverse*state.getGlobalLinkTransform(b);
      ta.translate(a->getCenteredBoundingBoxOffset());
      tb.translate(b->getCenteredBoundingBoxOffset());
      moveit::core::AABB first,second;
      first.extendWithTransformedBox(ta,a->getShapeExtentsAtOrigin());
      second.extendWithTransformedBox(tb,b->getShapeExtentsAtOrigin());
      const double lower=first.exteriorDistance(second);
      if (std::isfinite(lower) && lower > threshold+1e-9)
      {result.setEntry(a->getName(),b->getName(),true);break;}
    }
  }
  return result;
}

// The original collision checks always run. Only the distance search
// uses immutable geometry acceleration; unsupported scenes keep original FCL.
static bool supportsInitialDistance(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix& acm)
{
  std::vector<const moveit::core::AttachedBody*> attached;state.getAttachedBodies(attached);
  if(!attached.empty() || scene.getWorld()->size()!=0)return false;
  const auto& env=scene.getCollisionEnvUnpadded();
  if(!dynamic_cast<const collision_detection::CollisionEnvFCL*>(env.get()))return false;
  for(const auto& x:env->getLinkScale())if(x.second!=1.)return false;
  for(const auto& x:env->getLinkPadding())if(x.second!=0.)return false;
  const auto& links=state.getRobotModel()->getLinkModelsWithCollisionGeometry();
  for(size_t i=0;i<links.size();++i)for(size_t j=i+1;j<links.size();++j){
    collision_detection::AllowedCollision::Type type;
    if(acm.getAllowedCollision(links[i]->getName(),links[j]->getName(),type) &&
       type==collision_detection::AllowedCollision::CONDITIONAL)return false;
  }
  return true;
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
    distance_acm = boundedDistanceMatrix(state,environment,distance_acm,servo_params_.self_collision_proximity_threshold);
    collision_detection::DistanceRequest distance_request;
    distance_request.group_name = request.group_name;
    distance_request.enableGroup(state.getRobotModel());
    distance_request.enable_nearest_points = true;
    distance_request.acm = &distance_acm;
    // FCL's documented bounded query still computes every distance that can
    // change the original scale. At/above the unchanged threshold scale is 1.
    // SINGLE honors distance_threshold; GLOBAL replaces it with its current
    // minimum in the pinned MoveIt callback, so cannot be used for this bound.
    distance_request.type = collision_detection::DistanceRequestType::SINGLE;
    distance_request.distance_threshold = servo_params_.self_collision_proximity_threshold;
    collision_detection::DistanceResult distance_result;
    if(initial_distance_->valid() && supportsInitialDistance(scene,state,distance_acm))
    {
      SweepDistanceCertificate::WitnessCache witnesses;
      distance_result=initial_distance_->measure(scene,state,distance_acm,nullptr,nullptr,0.,
          nullptr,{},nullptr,distance_request.distance_threshold,nullptr,&witnesses);
      for(const auto& entry:witnesses){
        auto found=distance_result.distances.find(entry.first);
        if(found==distance_result.distances.end()||found->second.empty())continue;
        auto& d=found->second.front();const auto& w=entry.second;
        if(std::abs(d.distance-w.distance)>1e-9)continue;
        d.link_names[0]=entry.first.first;d.link_names[1]=entry.first.second;
        d.body_types[0]=d.body_types[1]=collision_detection::BodyType::ROBOT_LINK;
        d.nearest_points[0]=w.first_world;d.nearest_points[1]=w.second_world;
      }
      sample.initial_distance_optimized=true;
    }
    else environment->distanceSelf(distance_request, distance_result, state);
    for (const auto& item : distance_result.distances)
    {
      double minimum = std::numeric_limits<double>::infinity();
      const collision_detection::DistanceResultsData* witness=nullptr;
      for (const auto& entry : item.second)
        if(entry.distance<minimum){minimum=entry.distance;witness=&entry;}
      if (std::isfinite(minimum) && minimum > 0.0 && minimum < distance_request.distance_threshold)
      {
        const auto pair=std::minmax(item.first.first,item.first.second);
        sample.exact_self_distances[pair] = minimum;
        // Validate names, frames and finite point-distance agreement. Missing
        // witnesses merely select the original full-proof query order.
        if(witness && witness->body_types[0]==collision_detection::BodyType::ROBOT_LINK &&
           witness->body_types[1]==collision_detection::BodyType::ROBOT_LINK &&
           witness->nearest_points[0].allFinite() && witness->nearest_points[1].allFinite() &&
           std::abs((witness->nearest_points[0]-witness->nearest_points[1]).norm()-minimum)<1e-6)
        {
          const auto* a=state.getRobotModel()->getLinkModel(witness->link_names[0]);
          const auto* b=state.getRobotModel()->getLinkModel(witness->link_names[1]);
          if(a&&b && std::minmax(a->getName(),b->getName())==pair)
          {
            const auto pa=state.getGlobalLinkTransform(a).inverse()*witness->nearest_points[0];
            const auto pb=state.getGlobalLinkTransform(b).inverse()*witness->nearest_points[1];
            sample.exact_self_witnesses[pair]=a->getName()==pair.first?ProximityWitness{pa,pb}:ProximityWitness{pb,pa};
          }
        }
      }
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
