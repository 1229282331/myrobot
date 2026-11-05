// SPDX-License-Identifier: BSD-3-Clause
// Distance-only policy. Actual collisions always use the unmodified ACM.
#pragma once
#include <moveit/planning_scene/planning_scene.hpp>
#include <set>
#include <stdexcept>

class ArmProximityPolicy {
  moveit::core::RobotModelConstPtr model_;
  std::vector<std::pair<std::string,std::string>> invariant_pairs_;
public:
  ArmProximityPolicy(const moveit::core::RobotModelConstPtr& model,const std::string& group):model_(model) {
    const auto* joints=model->getJointModelGroup(group);
    if(!joints)throw std::runtime_error("Unknown proximity joint group");
    const auto& active=joints->getActiveJointModels();
    std::set<const moveit::core::JointModel*> selected(active.begin(),active.end());
    const auto& links=model->getLinkModelsWithCollisionGeometry();
    std::vector<std::vector<std::string>> signatures;
    for(const auto* link:links) {
      std::vector<std::string> signature;
      for(auto* ancestor=link;ancestor;ancestor=ancestor->getParentLinkModel()) {
        const auto* joint=ancestor->getParentJointModel();
        if(selected.count(joint))signature.push_back(joint->getName());
      }
      signatures.push_back(signature);
    }
    for(size_t i=0;i<links.size();++i)for(size_t j=i+1;j<links.size();++j)
      if(signatures[i]==signatures[j])invariant_pairs_.emplace_back(links[i]->getName(),links[j]->getName());
  }
  const auto& invariantPairs()const{return invariant_pairs_;}
  collision_detection::AllowedCollisionMatrix distanceMatrix(const collision_detection::AllowedCollisionMatrix& original)const {
    auto result=original;
    // Equal ordered ancestor signatures mean the selected group's motion
    // applies the same rigid transform to both links. Non-group joints (the
    // fingers) can still change their separation and retain collision checks.
    for(const auto& pair:invariant_pairs_)result.setEntry(pair.first,pair.second,true);
    return result;
  }
  void check(const collision_detection::CollisionEnvConstPtr& environment,
             const collision_detection::CollisionRequest& request,
             collision_detection::CollisionResult& result,
             const moveit::core::RobotState& state,
             const collision_detection::AllowedCollisionMatrix& original)const {
    auto collision_request=request;collision_request.distance=false;
    environment->checkSelfCollision(collision_request,result,state,original);
    if(result.collision){result.distance=0.;return;}
    auto proximity=distanceMatrix(original);
    collision_detection::DistanceRequest distance_request;
    distance_request.group_name=request.group_name;
    distance_request.enableGroup(model_);distance_request.acm=&proximity;
    collision_detection::DistanceResult distance_result;
    environment->distanceSelf(distance_request,distance_result,state);
    result.distance=distance_result.minimum_distance.distance;
    result.distance_result=distance_result;
  }
};
