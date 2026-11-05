// Minimal reproduction against MoveIt's actual acceleration plugin.
// No robot IO or transport. Requests use SI units and a fixed sample period.
#ifdef E6_REVIEWED_FILTER
#include "reviewed_acceleration_filter.hpp"
using SelectedFilter=e6_reviewed_smoothing::AccelerationLimitedPlugin;
#else
#include <moveit/online_signal_smoothing/acceleration_filter.hpp>
using SelectedFilter=online_signal_smoothing::AccelerationLimitedPlugin;
#endif
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

std::string read(const char* path) {
  std::ifstream in(path); if (!in) throw std::runtime_error("model missing");
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

int main(int argc, char** argv) {
  if (argc != 5) return 2;
  const double dt=std::stod(argv[3]), speed=std::stod(argv[4]);
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("acceleration_filter_reproduction");
  node->declare_parameter("planning_group_name", "me6_group");
  node->declare_parameter("update_period",dt);
  auto urdf=urdf::parseURDF(read(argv[1]));
  auto srdf=std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf,read(argv[2]))) return 3;
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto group=model->getJointModelGroup("me6_group");
  for (auto joint:model->getJointModels()) {
    if (!group->hasJointModel(joint->getName())) continue;
    auto bounds=joint->getVariableBoundsMsg();
    for (auto& bound:bounds) {bound.has_acceleration_limits=true;bound.max_acceleration=4*M_PI/180;}
    joint->setVariableBounds(bounds);
  }
  SelectedFilter filter;
  if (!filter.initialize(node,model,6)) return 4;
  Eigen::VectorXd q=Eigen::VectorXd::Zero(6),v=q,a=q;
  filter.reset(q,v,a);
  std::cout << std::setprecision(17);
  for (int step=0;step<int(8/dt);++step) {
    const double t=step*dt;
    const double requested=(t<3?speed:t<6?-speed:0)*M_PI/180;
    const auto old=q.eval();const auto old_v=v.eval();
    q[5]+=requested*dt;v.setZero();v[5]=requested;a.setZero();
    if (!filter.doSmoothing(q,v,a)) return 5;
    std::cout << "{\"t\":" << t << ",\"requested\":" << requested
              << ",\"q\":" << q[5] << ",\"v\":" << v[5]
              << ",\"dq_dt\":" << (q[5]-old[5])/dt
              << ",\"dv_dt\":" << (v[5]-old_v[5])/dt << "}\n";
  }
  rclcpp::shutdown();
}
