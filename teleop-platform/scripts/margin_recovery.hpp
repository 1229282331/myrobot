// SPDX-License-Identifier: BSD-3-Clause
// Escape from conservative finger reserves only. No device IO.
#pragma once
#include "sweep_distance.hpp"
#include <chrono>
#include <functional>

struct MarginRecoveryResult {
  bool attempted=false,verified=false;
  int joint=-1,pairs=0,low=-1,high=-1;
  double source_clearance_m=0.,outward_derivative_m_rad=0.,elapsed_us=0.;
  std::string reason="inactive";
};

class MarginRecovery {
  using Points=std::vector<Eigen::Vector3d>;
  struct Part {Points vertices;bool box=false;Points normals;};
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* arm_;
  SweepDistanceCertificate certificate_;
  std::map<std::string,std::vector<Part>> parts_;
  std::map<std::string,Points> sources_;
public:
  MarginRecoveryResult result;
  explicit MarginRecovery(moveit::core::RobotModelConstPtr model);
  bool assess(const planning_scene::PlanningScene& scene,const moveit::core::RobotState& actual,
      const Eigen::VectorXd& target,int low,int high,
      const std::function<void(moveit::core::RobotState&,int)>& aperture,
      std::chrono::steady_clock::time_point deadline);
};
