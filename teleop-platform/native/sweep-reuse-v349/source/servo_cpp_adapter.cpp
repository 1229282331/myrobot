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

// Device-independent official Servo adapter. The gateway alone owns motion IO.
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include "dynamic_gripper.hpp"
#include "collision_retreat.hpp"
#include "orientation_assist.hpp"

using namespace moveit_servo;
using Clock=std::chrono::steady_clock;
double condition(moveit::core::RobotState& state,const moveit::core::JointModelGroup* group) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(state.getJacobian(group));
  const auto sv=svd.singularValues();
  return sv.tail(1)(0)>1e-10?sv(0)/sv.tail(1)(0):std::numeric_limits<double>::infinity();
}
void scalarJson(double value) {if(std::isfinite(value))std::cout<<value;else std::cout<<"null";}
void vectorJson(const Eigen::VectorXd& v) {
  std::cout << '[';
  for(int i=0;i<v.size();++i){if(i)std::cout << ',';std::cout << v[i];}
  std::cout << ']';
}
void poseJson(moveit::core::RobotState& state) {
  const auto pose=state.getGlobalLinkTransform("controller_base").inverse()*state.getGlobalLinkTransform("controller_flange");
  const Eigen::Quaterniond rotation(pose.rotation());
  std::cout << "{\"position_m\":";vectorJson(pose.translation());
  std::cout << ",\"quaternion_xyzw\":";vectorJson(rotation.coeffs());std::cout << '}';
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("e6_cpp_adapter",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto listener=std::make_shared<const servo::ParamListener>(node,"moveit_servo");
  const auto params=listener->get_params();
  auto monitor=createPlanningSceneMonitor(node,params);
  std::unique_ptr<Servo> servo;
  auto publisher=node->create_publisher<sensor_msgs::msg::JointState>(params.joint_topic,1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]{executor.spin();});
  const std::vector<std::string> names={"joint1","joint2","joint3","joint4","joint5","joint6"};
  auto model=monitor->getRobotModel();
  auto state=std::make_shared<moveit::core::RobotState>(model);
  auto actual=std::make_shared<moveit::core::RobotState>(model);
  state->setToDefaultValues();actual->setToDefaultValues();
  auto group=model->getJointModelGroup(params.move_group_name);
  DynamicGripper gripper(node,monitor);
  CollisionRetreatPolicy retreat(model,params.move_group_name,params.self_collision_proximity_threshold);
  OrientationAssist assist;
  Eigen::VectorXd assist_lower(6),assist_upper(6);
  for(int i=0;i<6;++i){
    const auto& b=model->getVariableBounds(names[i]);
    assist_lower[i]=b.min_position_+params.joint_limit_margins.front();
    assist_upper[i]=b.max_position_-params.joint_limit_margins.front();
  }
  if(gripper.enabled)RCLCPP_INFO(node->get_logger(),"Retreat budget %d ms; gripper geometry budget %.0f ms; callback deadline 42 ms; cycle 50 ms; feedback/permit deadlines unchanged",CollisionRetreatPolicy::compute_budget_ms,DynamicGripper::maximum_budget_ms);
  const std::string control_frame=node->has_parameter("e6_control_frame")?
    node->get_parameter("e6_control_frame").as_string():"controller_flange";
  if(!model->getLinkModel(control_frame))throw std::runtime_error("Unknown TCP link");
  KinematicState target(6);target.joint_names=names;
  uint64_t active_epoch=0,previous_seq=0;
  bool initialized=false,was_active=false;
  std::cout << std::setprecision(17);
  std::string line;int result=0;
  try {
    while(rclcpp::ok() && std::getline(std::cin,line)) {
      const auto started=Clock::now();
      uint64_t seq,epoch;int active,kind,tool;
      Eigen::VectorXd q(6),v(6);
      std::istringstream input(line);
      if(!(input>>seq>>epoch>>active>>kind>>tool)||seq<=previous_seq)throw std::runtime_error("bad sequence/header");
      if((active!=0&&active!=1)||(kind<0||kind>2)||(tool!=0&&tool!=1))throw std::runtime_error("bad mode");
      for(int i=0;i<6;++i)if(!(input>>q[i])||!std::isfinite(q[i]))throw std::runtime_error("bad actual joints");
      for(int i=0;i<6;++i)if(!(input>>v[i])||!std::isfinite(v[i]))throw std::runtime_error("bad velocity");
      double assist_rate=0;
      if(kind==2){
        std::string marker;
        if(!(input>>marker>>assist_rate)||marker!="A1"||!std::isfinite(assist_rate)||assist_rate<=0||
           assist_rate>OrientationAssist::speed_limit+1e-12||v.tail<3>().norm()>0)
          throw std::runtime_error("bad orientation assistance request");
      }
      int grip_raw=-1,grip_sent=-1,grip_inflight=-1,grip_requested=-1,grip_hold=-1;
      if(gripper.enabled) {
        std::string marker;
        if(!(input>>marker>>grip_raw>>grip_sent>>grip_inflight>>grip_requested)||(marker!="G2"&&marker!="G3"))throw std::runtime_error("missing dynamic gripper input");
        if(marker=="G3" && (!(input>>grip_hold)||grip_hold<0||grip_hold>2))throw std::runtime_error("bad gripper hold state");
        for(auto r:{grip_raw,grip_sent,grip_inflight,grip_requested})if(r < -1 || r > 255)throw std::runtime_error("bad gripper count");
      }
      std::string extra;if(input>>extra)throw std::runtime_error("extra input");
      previous_seq=seq;
      sensor_msgs::msg::JointState feedback;
      feedback.header.stamp=node->now();feedback.name=names;
      feedback.position.assign(q.data(),q.data()+6);gripper.feedback(feedback,grip_raw);publisher->publish(feedback);
      const auto feedback_updated=Clock::now();
      // Initialize the upstream monitor and plugin once. Subsequent request
      // transforms and collision checks both use `actual`, filled from THIS
      // request below; DDS publication is observation, not a second feedback
      // authority. Source-feedback and callback deadlines remain unchanged.
      if(!initialized && !monitor->getStateMonitor()->waitForCurrentState(feedback.header.stamp,2.0))
        throw std::runtime_error("actual state delivery timeout");
      const auto state_delivered=Clock::now();
      if(!initialized){servo=std::make_unique<Servo>(node,listener,monitor,CollisionCheckMode::REQUEST_STATE);initialized=true;}
      const bool reset=!active||!was_active||epoch!=active_epoch;
      retreat.reset();
      if(reset){target.positions=q;target.velocities.setZero();target.accelerations.setZero();servo->resetSmoothing(target);active_epoch=epoch;}
      const KinematicState seed=target;
      actual->setJointGroupPositions(group,q);actual->update();
      gripper.position(*actual,grip_raw);gripper.position(*state,grip_raw);
      state->setJointGroupPositions(group,target.positions);
      state->setJointGroupVelocities(group,target.velocities);state->update();
      const double condition_before=condition(*state,group);
      const auto tcp_rotation=[&](const moveit::core::RobotState& s)->Eigen::Matrix3d {
        return s.getGlobalLinkTransform("controller_base").rotation().transpose()*s.getGlobalLinkTransform(control_frame).rotation();
      };
      const Eigen::Matrix3d seed_rotation=tcp_rotation(*state);
      if(kind==2&&active)assist.begin(epoch,tcp_rotation(*actual));
      else assist.stop();
      double condition_target=condition_before;
      bool retreat_rejected=false;
      double compute_us=0;int status=0;
      if(active) {
        // An escape only uses known, stationary gripper geometry. Any current
        // or proposed finger motion keeps the original proximity scaling.
        const bool observed_idle=grip_sent<0 && (grip_requested<0 || std::abs(grip_raw-grip_requested)<=1);
        const bool held=grip_sent>=0 && std::abs(grip_raw-grip_sent)<=1 && grip_requested==grip_sent;
        const bool same_commitment=grip_inflight<0 || (grip_sent>=0&&grip_inflight==grip_sent);
        const bool frozen_contact=grip_hold==2 && grip_sent>=0 && grip_requested==grip_sent;
        const bool retreat_eligible=gripper.enabled && grip_raw>=0 && same_commitment &&
          ((grip_hold==1 && (observed_idle||held)) || frozen_contact ||
           (grip_hold<0 && grip_inflight<0 && (observed_idle||held)));
        const auto hold_excursion=grip_hold>=0?gripper.holdExcursion(grip_raw,grip_sent,grip_inflight):std::map<std::string,double>{};
        const auto retreat_deadline=started+std::chrono::milliseconds(CollisionRetreatPolicy::compute_budget_ms);
        servo->setCollisionScaleSelector([&](double baseline,const Eigen::VectorXd& smoothed){
          retreat.result.baseline_scale=retreat.result.selected_scale=baseline;
          if(!retreat_eligible){retreat.result.reason="gripper_not_stationary";return baseline;}
          retreat.result.attempted=true;
          planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
          retreat.reuseCheckedAnchor(*scene.operator->(),*actual,servo->getCollisionSample());
          if(retreat.assess(*scene.operator->(),*actual,smoothed,retreat_deadline,Clock::time_point::min(),hold_excursion)) {
            retreat.result.boosted=true;retreat.result.verified=true;retreat.result.selected_scale=1.;return 1.;
          }
          return baseline;
        });
        servo->setCommandType(kind==0?CommandType::JOINT_JOG:CommandType::TWIST);
        ServoInput command;
        if(kind==0)command=JointJogCommand{names,std::vector<double>(v.data(),v.data()+6)};
        else if(kind==2){
          Eigen::MatrixXd jacobian;
          if(!state->getJacobian(group,model->getLinkModel(control_frame),Eigen::Vector3d::Zero(),jacobian))
            throw std::runtime_error("Missing grasp TCP Jacobian");
          const auto parent=group->getJointModels().front()->getParentLinkModel();
          const Eigen::Matrix3d basis=state->getGlobalLinkTransform("controller_base").rotation().transpose()*
            state->getGlobalLinkTransform(parent).rotation();
          jacobian.topRows(3)=basis*jacobian.topRows(3).eval();
          jacobian.bottomRows(3)=basis*jacobian.bottomRows(3).eval();
          const auto strict_escape=[&](const Eigen::VectorXd& fixed){
            if(condition_before<params.lower_singularity_threshold || !fixed.allFinite() || fixed.norm()<1e-12)return false;
            // Read-only differential FK probe, at most 0.0001 rad in joint
            // space. It classifies direct retreat; it cannot authorize motion.
            moveit::core::RobotState probe(*state);
            const Eigen::VectorXd test_q=target.positions+fixed*std::min(.01,1e-4/fixed.norm());
            probe.setJointGroupPositions(group,test_q);probe.update();
            const double predicted=condition(probe,group);
            return std::isfinite(predicted)&&predicted<condition_before-1e-8;
          };
          command=TwistCommand{"controller_base",assist.suggest(jacobian,target.positions,assist_lower,assist_upper,
              seed_rotation,v.head<3>(),tool,assist_rate,condition_before,strict_escape)};
        }
        else command=TwistCommand{tool?control_frame:"controller_base",v};
        const auto begin=Clock::now();
        auto computed=servo->getNextJointState(state,command,actual);
        servo->setCollisionScaleSelector({});
        compute_us=std::chrono::duration<double,std::micro>(Clock::now()-begin).count();
        status=static_cast<int>(servo->getStatus());
        if(status==0||status==1||status==3||status==4) {
          state->setJointGroupPositions(group,computed.positions);state->update();
          condition_target=condition(*state,group);
          // Verify the actual official smoothed target, not just a requested
          // twist projection. Inertia must not carry an escape deeper.
          if(kind!=0 && condition_before>=params.hard_stop_singularity_threshold &&
             (!std::isfinite(condition_target)||condition_target>=condition_before-1e-8)) {
            status=2;retreat_rejected=true;
          }
          if(kind==2&&!assist.accepts(seed_rotation,tcp_rotation(*state))){
            status=6;assist.reason="final_angular_envelope";
          }
        }
        // Official INVALID/collision output is not an executable zero target.
        // Reset the smoother before any future command; the host reports the
        // boundary and the gateway independently stops the real arm.
        if(status==-1||status==2||status==5||status==6) {
          target.positions=q;target.velocities.setZero();target.accelerations.setZero();
          servo->resetSmoothing(target);was_active=false;
        }else{
          if(!computed.positions.allFinite()||!computed.velocities.allFinite())throw std::runtime_error("nonfinite target");
          target=computed;was_active=true;
        }
      }else was_active=false;
      const auto grip_arm_target=target.positions;
      auto grip_check=gripper.check(*actual,target.positions,group,grip_raw,grip_sent,grip_inflight,grip_requested,
                                    started+std::chrono::milliseconds(42),
                                    [&](const auto& scene,const auto& anchor,const auto& acm){return retreat.anchorSeparations(scene,anchor,acm);});
      if(active && Clock::now()>=started+std::chrono::milliseconds(42)){
        grip_check.arm_safe=false;grip_check.allowed=false;grip_check.budget_exhausted=true;
        grip_check.arm_reason="callback_deadline";
      }
      if(gripper.enabled && active && !grip_check.arm_safe){
        status=5;target.positions=q;target.velocities.setZero();target.accelerations.setZero();
        servo->resetSmoothing(target);was_active=false;
      }
      std::cout << "{\"seq\":"<<seq<<",\"epoch\":"<<epoch<<",\"active\":"<<(active?"true":"false")
        <<",\"reset\":"<<(reset?"true":"false")<<",\"status\":"<<status<<",\"compute_us\":"<<compute_us
        <<",\"feedback_update_us\":"<<std::chrono::duration<double,std::micro>(feedback_updated-started).count()
        <<",\"state_wait_us\":"<<std::chrono::duration<double,std::micro>(state_delivered-feedback_updated).count()
        <<",\"callback_us\":"<<std::chrono::duration<double,std::micro>(Clock::now()-started).count()
        <<",\"seed_positions\":";vectorJson(seed.positions);
      std::cout<<",\"seed_velocities\":";vectorJson(seed.velocities);
      std::cout<<",\"joints_rad\":";vectorJson(target.positions);
      std::cout<<",\"velocities_rad_s\":";vectorJson(target.velocities);
      std::cout<<",\"actual_fk\":";poseJson(*actual);
      state->setJointGroupPositions(group,target.positions);state->update();
      std::cout<<",\"target_fk\":";poseJson(*state);
      std::cout<<",\"orientation_assist\":{\"version\":1,\"enabled\":"<<(kind==2&&active?"true":"false")
        <<",\"reason\":\""<<assist.reason<<"\",\"max_angle_deg\":15,\"max_rate_rad_s\":"<<assist_rate
        <<",\"deviation_deg\":"<<(kind==2&&active?OrientationAssist::angle(assist.anchor,tcp_rotation(*state))*180/OrientationAssist::pi:0.)
        <<",\"anchor_xyzw\":";vectorJson(Eigen::Quaterniond(assist.anchor).coeffs());
      std::cout<<",\"twist_base\":";vectorJson(assist.twist);std::cout<<"}";
      std::cout<<",\"singularity\":{\"condition_before\":";scalarJson(condition_before);
      std::cout<<",\"condition_target\":";scalarJson(condition_target);
      std::cout<<",\"retreat_rejected\":"<<(retreat_rejected?"true":"false")<<"}";
      const auto& cr=retreat.result;
      std::cout<<",\"collision_retreat\":{\"attempted\":"<<(cr.attempted?"true":"false")
        <<",\"boosted\":"<<(cr.boosted?"true":"false")<<",\"verified\":"<<(cr.verified?"true":"false")
        <<",\"baseline_scale\":"<<cr.baseline_scale<<",\"selected_scale\":"<<cr.selected_scale
        <<",\"reason\":\""<<cr.reason<<"\",\"evaluations\":"<<cr.evaluations<<",\"samples\":"<<cr.samples
        <<",\"identical_path_reuses\":"<<cr.identical_path_reuses
        <<",\"elapsed_us\":"<<cr.elapsed_us<<",\"travel_m\":"<<cr.travel_m
        <<",\"actual_collision_us\":"<<cr.actual_collision_us<<",\"initial_distances_us\":"<<cr.initial_distances_us
        <<",\"sample_queries_us\":"<<cr.sample_queries_us<<",\"interval_checks_us\":"<<cr.interval_checks_us
        <<",\"minimum_before_m\":"<<cr.minimum_before_m<<",\"minimum_after_m\":"<<cr.minimum_after_m
        <<",\"budget_exhausted\":"<<(cr.budget_exhausted?"true":"false")<<"}";
      if(gripper.enabled){
        std::cout<<",\"gripper_check\":{\"allowed\":"<<(grip_check.allowed?"true":"false")
          <<",\"arm_safe\":"<<(grip_check.arm_safe?"true":"false")<<",\"fallback\":"<<(grip_check.fallback?"true":"false")
          <<",\"hold_state\":"<<grip_hold<<",\"actual_raw\":"<<grip_raw<<",\"requested_raw\":"<<grip_requested
          <<",\"sent_raw\":"<<grip_sent<<",\"inflight_raw\":"<<grip_inflight
          <<",\"range_raw\":["<<grip_check.low<<","<<grip_check.high<<"],\"checks\":"<<grip_check.checks
          <<",\"arm_range_raw\":["<<grip_check.arm_low<<","<<grip_check.arm_high<<"]"
          <<",\"budget_ms\":"<<grip_check.budget_ms<<",\"elapsed_us\":"<<grip_check.elapsed_us<<",\"arm_q_rad\":";vectorJson(q);
        std::cout<<",\"budget_exhausted\":"<<(grip_check.budget_exhausted?"true":"false");
        std::cout<<",\"arm_reason\":\""<<grip_check.arm_reason<<"\",\"arm_travel_m\":";scalarJson(grip_check.arm_travel_m);
        std::cout<<",\"arm_target_rad\":";vectorJson(grip_arm_target);std::cout<<"}";
      }
      if(active){
        const auto& sample=servo->getCollisionSample();
        std::cout<<",\"collision_snapshot\":{\"scale\":"<<sample.scale
          <<",\"checked\":"<<(sample.checked?"true":"false")
          <<",\"self_collision\":"<<(sample.self_collision?"true":"false")
          <<",\"scene_collision\":"<<(sample.scene_collision?"true":"false")<<",\"self_distance_m\":";
        scalarJson(sample.self_distance);std::cout<<",\"scene_distance_m\":";scalarJson(sample.scene_distance);std::cout<<"}";
      }
      std::cout<<"}"<<std::endl;
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=2;}
  servo.reset();executor.cancel();spinner.join();rclcpp::shutdown();return result;
}
