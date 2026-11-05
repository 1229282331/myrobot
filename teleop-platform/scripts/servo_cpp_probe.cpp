/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, PickNik Inc.
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

// Isolated latency probe adapted from the pinned official C++ Servo demo.
// No Dobot SDK, device socket, CAN or actuator publisher. stdin is test input.
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

using namespace moveit_servo;
using Clock=std::chrono::steady_clock;
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("e6_cpp_probe",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto listener=std::make_shared<const servo::ParamListener>(node,"moveit_servo");
  const auto params=listener->get_params();
  auto monitor=createPlanningSceneMonitor(node,params);
  // The official constructor waits for a complete JointState. Start the
  // fixture publisher/executor first; no live robot publishes in this probe.
  std::unique_ptr<Servo> servo;
  auto publisher=node->create_publisher<sensor_msgs::msg::JointState>(params.joint_topic,1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]{executor.spin();});
  std::vector<std::string> names={"joint1","joint2","joint3","joint4","joint5","joint6"};
  auto model=monitor->getRobotModel();
  auto state=std::make_shared<moveit::core::RobotState>(model);
  state->setToDefaultValues();
  auto group=model->getJointModelGroup(params.move_group_name);
  KinematicState target(6);target.joint_names=names;
  uint64_t active_epoch=0;
  bool initialized=false,was_active=false;
  std::cout<<std::setprecision(17);
  std::string line;
  int result=0;
  try {
    while(rclcpp::ok() && std::getline(std::cin,line)) {
      auto start=Clock::now();
      uint64_t seq,epoch;int active,kind,tool;
      Eigen::VectorXd q(6),v(6);
      std::istringstream input(line);
      if(!(input>>seq>>epoch>>active>>kind>>tool))throw std::runtime_error("bad header");
      if((active!=0&&active!=1)||(kind!=0&&kind!=1)||(tool!=0&&tool!=1))throw std::runtime_error("bad mode");
      for(int i=0;i<6;++i)if(!(input>>q[i])||!std::isfinite(q[i]))throw std::runtime_error("bad q");
      for(int i=0;i<6;++i)if(!(input>>v[i])||!std::isfinite(v[i]))throw std::runtime_error("bad v");
      std::string extra;if(input>>extra)throw std::runtime_error("extra input");
      sensor_msgs::msg::JointState feedback;
      feedback.header.stamp=node->now();feedback.name=names;
      feedback.position.assign(q.data(),q.data()+6);publisher->publish(feedback);
      if(!initialized) {
        if(!monitor->getStateMonitor()->waitForCurrentState(feedback.header.stamp,2.0))throw std::runtime_error("fixture state unavailable");
        servo=std::make_unique<Servo>(node,listener,monitor);
        initialized=true;
      }
      // Same target-state propagation as the official C++ demos. The test
      // plant separately reports actual q; it is never a real-device observer.
      if(!active||!was_active||epoch!=active_epoch) {
        target.positions=q;target.velocities.setZero();target.accelerations.setZero();
        servo->resetSmoothing(target);active_epoch=epoch;
      }
      state->setJointGroupPositions(group,target.positions);
      state->setJointGroupVelocities(group,target.velocities);state->update();
      double compute_us=0;
      if(active) {
        servo->setCommandType(kind==0?CommandType::JOINT_JOG:CommandType::TWIST);
        ServoInput command;
        if(kind==0)command=JointJogCommand{names,std::vector<double>(v.data(),v.data()+6)};
        else command=TwistCommand{tool?"controller_flange":"controller_base",v};
        const auto begin=Clock::now();
        target=servo->getNextJointState(state,command);
        compute_us=std::chrono::duration<double,std::micro>(Clock::now()-begin).count();
        if(!target.positions.allFinite())throw std::runtime_error("nonfinite target");
      }
      was_active=active;
      std::cout<<"{\"seq\":"<<seq<<",\"epoch\":"<<epoch<<",\"active\":"<<(active?"true":"false")
               <<",\"status\":"<<static_cast<int>(servo->getStatus())<<",\"compute_us\":"<<compute_us
               <<",\"callback_us\":"<<std::chrono::duration<double,std::micro>(Clock::now()-start).count()
               <<",\"joints_rad\":[";
      for(int i=0;i<6;++i){if(i)std::cout<<',';std::cout<<target.positions[i];}
      std::cout<<"]}"<<std::endl;
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=2;}
  servo.reset();executor.cancel();spinner.join();rclcpp::shutdown();return result;
}
