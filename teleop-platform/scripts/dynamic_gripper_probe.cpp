// Read-only isolated replay of production swept collision checks, no device IO.
#include <moveit_servo/utils/common.hpp>
#include <moveit_servo/servo.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "dynamic_gripper.hpp"
int main(int argc,char**argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("e6_cpp_adapter",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 auto listener=std::make_shared<const servo::ParamListener>(node,"moveit_servo");
 auto params=listener->get_params();auto monitor=moveit_servo::createPlanningSceneMonitor(node,params);
 auto model=monitor->getRobotModel();auto arm=model->getJointModelGroup(params.move_group_name);
 DynamicGripper gripper(node,monitor);moveit::core::RobotState state(model);state.setToDefaultValues();
 std::cout<<std::setprecision(17);std::string line;
 while(std::getline(std::cin,line)) {
  std::istringstream in(line);Eigen::VectorXd q(6),target(6);int raw,sent,inflight,requested;
  for(int i=0;i<6;++i)in>>q[i];for(int i=0;i<6;++i)in>>target[i];in>>raw>>sent>>inflight>>requested;
  if(!in || !q.allFinite() || !target.allFinite() || raw< -1 || raw>255)return 3;
  double remaining_ms=1e6;in>>remaining_ms;
  if(!std::isfinite(remaining_ms)||remaining_ms<0)return 3;
  state.setJointGroupPositions(arm,q);gripper.position(state,raw);
  sensor_msgs::msg::JointState feedback;gripper.feedback(feedback,raw);
  auto deadline=std::chrono::steady_clock::now()+std::chrono::microseconds(static_cast<int64_t>(remaining_ms*1000));
  auto r=gripper.check(state,target,arm,raw,sent,inflight,requested,deadline);
  std::cout<<"{\"arm_safe\":"<<r.arm_safe<<",\"allowed\":"<<r.allowed<<",\"budget_exhausted\":"<<r.budget_exhausted
   <<",\"budget_ms\":"<<r.budget_ms<<",\"arm_reason\":\""<<r.arm_reason<<"\""
   <<",\"elapsed_us\":"<<r.elapsed_us<<",\"checks\":"<<r.checks<<"}"<<std::endl;
 }
 rclcpp::shutdown();return 0;
}
