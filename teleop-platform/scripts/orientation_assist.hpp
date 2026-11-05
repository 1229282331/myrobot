// Bounded angular suggestion only. Official Servo still performs IK, velocity
// limiting, singularity handling and smoothing; the existing FCL proof gates
// the final target. No IO and no permission to override any stop.
#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <functional>

class OrientationAssist {
public:
  static constexpr double pi=3.14159265358979323846;
  static constexpr double angle_limit=15*pi/180;
  static constexpr double speed_limit=3*pi/180;
  static constexpr double dt=.05;
  bool initialized=false;
  uint64_t epoch=0;
  Eigen::Matrix3d anchor=Eigen::Matrix3d::Identity();
  Eigen::Vector3d stage1=Eigen::Vector3d::Zero(),stage2=stage1;
  Eigen::Matrix<double,6,1> twist=Eigen::Matrix<double,6,1>::Zero();
  double pressure=0.,deviation=0.,requested_limit=0.;
  std::string reason="inactive";

  static double angle(const Eigen::Matrix3d& from,const Eigen::Matrix3d& to) {
    return Eigen::AngleAxisd(to*from.transpose()).angle();
  }
  void begin(uint64_t id,const Eigen::Matrix3d& actual) {
    if(!initialized||epoch!=id){initialized=true;epoch=id;anchor=actual;stage1.setZero();stage2.setZero();}
  }
  void stop() {stage1.setZero();stage2.setZero();twist.setZero();reason="inactive";}
  // J maps joint rates to the grasp TCP twist, expressed in controller_base.
  // Translation stays exactly the requested vector. Choose an angular rate
  // that reduces weighted joint-rate effort, with rotation penalized. This is
  // local velocity assistance, not a global planner or reachable-set claim.
  Eigen::Matrix<double,6,1> suggest(const Eigen::MatrixXd& J,
      const Eigen::VectorXd& q,const Eigen::VectorXd& lower,const Eigen::VectorXd& upper,
      const Eigen::Matrix3d& current,const Eigen::Vector3d& linear,bool tool,double max_rate,
      double condition,const std::function<bool(const Eigen::VectorXd&)>& strict_escape={});
  bool accepts(const Eigen::Matrix3d& seed,const Eigen::Matrix3d& target) const {
    return angle(anchor,target)<=angle_limit+1e-9 &&
           angle(seed,target)<=requested_limit*dt+1e-7;
  }
};
