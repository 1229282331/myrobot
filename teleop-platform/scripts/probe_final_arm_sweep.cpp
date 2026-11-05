// Read-only reconstruction of saved accepted candidates against original FCL.
#include "dynamic_gripper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>

static void contacts(std::ostream& out,const collision_detection::CollisionResult& result) {
  out<<"[";bool comma=false;
  for(const auto& pair:result.contacts){
    if(comma)out<<",";comma=true;
    out<<"[\""<<pair.first.first<<"\",\""<<pair.first.second<<"\"]";
  }
  out<<"]";
}
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("e6_target_collision_probe",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto monitor=std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node,"robot_description");
  auto model=monitor->getRobotModel();
  auto* group=model->getJointModelGroup("me6_group");
  DynamicGripper gripper(node,monitor);
  moveit::core::RobotState actual(model);actual.setToDefaultValues();
  std::string line;size_t index=0;
  std::cout<<std::setprecision(17);
  while(std::getline(std::cin,line)) {
    std::istringstream input(line);Eigen::VectorXd q(6),target(6);int raw;
    for(auto& x:q)if(!(input>>x))return 2;
    for(auto& x:target)if(!(input>>x))return 3;
    if(!(input>>raw))return 4;
    actual.setJointGroupPositions(group,q);actual.update();gripper.position(actual,raw);
    sensor_msgs::msg::JointState feedback;gripper.feedback(feedback,raw);
    const auto check=gripper.check(actual,target,group,raw,-1,-1,raw);
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
    collision_detection::CollisionRequest req;req.group_name="me6_group";
    req.contacts=true;req.max_contacts=100;
    collision_detection::CollisionResult start,end,fingers;
    scene->checkSelfCollision(req,start,actual);
    auto pose=actual;pose.setJointGroupPositions(group,target);pose.update();
    scene->checkSelfCollision(req,end,pose);
    req.group_name="e6_gripper_geometry";scene->checkSelfCollision(req,fingers,pose);
    req.group_name="me6_group";
    int colliding=0;double first=-1.;
    for(int i=0;i<=64;++i) {
      pose.setJointGroupPositions(group,q+(target-q)*(double(i)/64));pose.update();
      collision_detection::CollisionResult result;scene->checkSelfCollision(req,result,pose);
      if(result.collision){++colliding;if(first<0)first=double(i)/64;}
    }
    std::cout<<"{\"index\":"<<index++<<",\"arm_safe\":"<<(check.arm_safe?"true":"false")
      <<",\"budget_exhausted\":"<<(check.budget_exhausted?"true":"false")
      <<",\"reason\":\""<<check.arm_reason<<"\",\"actual_collision\":"<<(start.collision?"true":"false")
      <<",\"target_collision\":"<<(end.collision?"true":"false")
      <<",\"finger_target_collision\":"<<(fingers.collision?"true":"false")
      <<",\"colliding_samples\":"<<colliding<<",\"first_fraction\":"<<first
      <<",\"target_contacts\":";contacts(std::cout,end);std::cout<<"}"<<std::endl;
  }
  rclcpp::shutdown();return 0;
}
