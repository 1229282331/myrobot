// Geometry-only extension to the official MoveIt/FCL adapter. No device IO.
#pragma once
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <algorithm>
#include "sweep_distance.hpp"

struct GripperSweepResult {
  bool enabled=false,allowed=false,arm_safe=true,fallback=true,budget_exhausted=false;
  int raw=-1,sent=-1,inflight=-1,requested=-1,low=-1,high=-1,checks=0;
  int arm_low=-1,arm_high=-1;
  double elapsed_us=0,arm_travel_m=0,budget_ms=0;
  std::string arm_reason;
};

class DynamicGripper {
  std::vector<std::string> names,links;
  std::vector<double> poses;
  collision_detection::AllowedCollisionMatrix original;
  std::string fallback;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor;
  SweepDistanceCertificate certificate;
public:
  // Geometry gets 5 ms more headroom than v271. The adapter additionally caps
  // the WHOLE callback at 42 ms, leaving 8 ms in its unchanged 50 ms cycle.
  // Neither the original feedback deadline nor permit age is extended.
  static constexpr double maximum_budget_ms=35.;
  bool enabled=false;
  DynamicGripper(const rclcpp::Node::SharedPtr& node, planning_scene_monitor::PlanningSceneMonitorPtr m):monitor(m),certificate(m->getRobotModel()) {
    enabled=node->has_parameter("e6_gripper_joint_names");
    if(!enabled)return;
    names=node->get_parameter("e6_gripper_joint_names").as_string_array();
    links=node->get_parameter("e6_gripper_links").as_string_array();
    poses=node->get_parameter("e6_gripper_poses").as_double_array();
    fallback=node->get_parameter("e6_gripper_fallback").as_string();
    if(names.size()!=8 || poses.size()!=256*8)throw std::runtime_error("Invalid gripper pose table");
    for(auto& name:names)if(!m->getRobotModel()->getJointModel(name))throw std::runtime_error("Missing gripper joint");
    planning_scene_monitor::LockedPlanningSceneRO scene(m);original=scene->getAllowedCollisionMatrix();
  }
  void position(moveit::core::RobotState& state,int raw)const {
    if(!enabled)return;
    raw=std::clamp(raw,0,255);
    for(size_t i=0;i<names.size();++i)state.setVariablePosition(names[i],poses[raw*8+i]);
    state.update();
  }
  std::map<std::string,double> holdExcursion(int raw,int sent,int inflight)const {
    std::map<std::string,double> result;
    if(!enabled||raw<0)return result;
    const int lo=std::max(0,std::min({raw-1,sent<0?raw:sent,inflight<0?raw:inflight}));
    const int hi=std::min(255,std::max({raw+1,sent<0?raw:sent,inflight<0?raw:inflight}));
    for(size_t i=0;i<names.size();++i)for(int r=lo;r<=hi;++r)
      result[names[i]]=std::max(result[names[i]],std::abs(poses[r*8+i]-poses[raw*8+i]));
    return result;
  }
  void feedback(sensor_msgs::msg::JointState& message,int raw) {
    if(!enabled)return;
    // Unknown feedback is explicitly represented by the conservative proxy.
    // Zero values below only keep ROS joint completeness; not measured state.
    for(size_t i=0;i<names.size();++i){message.name.push_back(names[i]);message.position.push_back(poses[std::max(raw,0)*8+i]);}
    auto acm=original;
    if(raw<0)for(auto& link:links)acm.setEntry(link,true);
    else acm.setEntry(fallback,true);
    // Most updates change only joint feedback, not the collision matrix.
    // Avoid an exclusive writer on every frame while the collision monitor
    // is reading. Compare actual contents so external changes are not missed.
    moveit_msgs::msg::AllowedCollisionMatrix desired,current;
    acm.getMessage(desired);
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
      scene->getAllowedCollisionMatrix().getMessage(current);
      if(current==desired)return;
    }
    planning_scene_monitor::LockedPlanningSceneRW scene(monitor);
    scene->getAllowedCollisionMatrixNonConst()=acm;
  }
  GripperSweepResult check(const moveit::core::RobotState& actual,const Eigen::VectorXd& target,
                          const moveit::core::JointModelGroup* arm,int raw,int sent,int inflight,int requested,
                          std::chrono::steady_clock::time_point callback_deadline=std::chrono::steady_clock::time_point::max(),
                          const SweepDistanceCertificate::AnchorDistanceSource& anchor_distances={}) {
    GripperSweepResult out;out.enabled=enabled;if(!enabled)return out;
    auto started=std::chrono::steady_clock::now();
    const auto deadline=std::min(callback_deadline,started+std::chrono::milliseconds(35));
    out.budget_ms=std::max(0.,std::chrono::duration<double,std::milli>(deadline-started).count());
    auto expired=[&]() {
      if(std::chrono::steady_clock::now()>=deadline){out.budget_exhausted=true;return true;}
      return false;
    };
    out.raw=raw;out.sent=sent;out.inflight=inflight;out.requested=requested;out.fallback=raw<0;
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
    auto acm=original;
    if(raw<0)for(auto& link:links)acm.setEntry(link,true);
    else acm.setEntry(fallback,true);
    collision_detection::CollisionRequest req;
    // This group includes the fingers and fallback proxy, checked against ALL
    // other links. Static E6 self collision remains the official Servo monitor.
    // MoveIt group filtering uses UPDATED links, not all explicitly listed
    // links. The fixed fallback proxy is not updated by the gripper joints,
    // so unknown-state queries must use the whole robot.
    req.group_name=raw<0?"":"e6_gripper_geometry";
    std::vector<double> seed;actual.copyJointGroupPositions(arm,seed);
    Eigen::Map<Eigen::VectorXd> initial(seed.data(),seed.size());
    collision_detection::DistanceResult distances;
    bool can_certify=raw>=0 && certificate.valid();
    std::vector<const moveit::core::AttachedBody*> attached;actual.getAttachedBodies(attached);
    can_certify=can_certify && attached.empty();
    auto excursions=[&](int low,int high) {
        std::vector<double> excursion(certificate.variables.size(),0.);
        const auto& arm_names=arm->getVariableNames();
        for(size_t v=0;v<certificate.variables.size();++v) {
          const auto& name=certificate.variables[v];
          auto ai=std::find(arm_names.begin(),arm_names.end(),name);
          if(ai!=arm_names.end())excursion[v]=std::abs(target[ai-arm_names.begin()]-initial[ai-arm_names.begin()]);
          auto gi=std::find(names.begin(),names.end(),name);
          if(gi!=names.end())for(int r=low;r<=high;++r)
            excursion[v]=std::max(excursion[v],std::abs(poses[r*8+(gi-names.begin())]-poses[raw*8+(gi-names.begin())]));
        }
        return excursion;
    };
    const int low=raw<0?0:std::min({std::max(0,raw-1),sent<0?raw:sent,inflight<0?raw:inflight,requested<0?raw:requested});
    const int high=raw<0?255:std::max({std::min(255,raw+1),sent,inflight,requested});
    const double reach[6]={1.,.9,.7,.5,.4,.3};double travel=0;
    for(int i=0;i<6;++i)travel+=std::abs(initial[i]-target[i])*reach[i];
    if(certificate.valid())travel=raw<0?certificate.armTravel(arm,initial,target):
      certificate.armTravel(arm,actual,initial,target,excursions(low,high));
    // Preserve the 40 mm maximum point travel and 1 mm sweep spacing.
    out.arm_travel_m=travel;
    if(!std::isfinite(travel)||travel>.04){
      out.arm_safe=false;out.arm_reason="sweep_travel_limit";
      out.elapsed_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-started).count();return out;
    }
    int arm_steps=int(std::ceil(travel/.001));
    if(can_certify) {
      const auto* known=anchor_distances?anchor_distances(*scene.operator->(),actual,acm):nullptr;
      distances=certificate.distances(*scene.operator->(),actual,acm,excursions(low,high),0.,nullptr,nullptr,{},known);++out.checks;
    }
    auto sweep=[&](int low,int high) {
      if(low>high)return true;
      if(expired())return false;
      auto narrow_acm=acm;
      if(can_certify) {
        // The angle extrema include every table entry; between entries the
        // nominal model is linear. This certifies ALL arm/aperture combinations
        // in the interval. Close pairs retain the original dense FCL fallback.
        if(certificate.certifies(distances,acm,excursions(low,high),actual))return !expired();
        narrow_acm=certificate.refinedMatrix(distances,acm,excursions(low,high),actual);
      }
      moveit::core::RobotState test(actual);
      const auto pairs=can_certify?certificate.preparePairs(*scene.operator->(),req,actual,narrow_acm):SweepDistanceCertificate::PreparedPairs{};
      // One raw count is <= 0.177 degrees at the driver. The generated coverage
      // proof bounds every collider corner's intervening travel below 1 mm.
      for(int a=0;a<=arm_steps;++a){
        test.setJointGroupPositions(arm,initial+(target-initial)*(arm_steps?double(a)/arm_steps:0.));
        for(int r=low;r<=high;++r){
          // Reserve time for the official Servo step and IPC. New gripper
          // proposals never consume an unbounded Cartesian-product sweep.
          if(expired())return false;
          position(test,r);collision_detection::CollisionResult result;
          if(!certificate.checkPairs(pairs,test,result))
            scene->checkSelfCollision(req,result,test,narrow_acm);
          ++out.checks;
          if(expired()||result.collision)return false;
        }
      }
      return !expired();
    };
    if(raw<0){
      out.low=-1;out.high=-1;out.arm_safe=out.allowed=sweep(0,0);
      if(!out.arm_safe)out.arm_reason=out.budget_exhausted?"sweep_budget":"collision";
    }
    else {
      int lo=std::min({raw,sent<0?raw:sent,inflight<0?raw:inflight});
      int hi=std::max({raw,sent<0?raw:sent,inflight<0?raw:inflight});
      out.low=std::min(lo,requested<0?raw:requested);out.high=std::max(hi,requested<0?raw:requested);
      // Existing commitments take priority; waiting on a new gripper target
      // must not withhold an independently valid arm step.
      out.arm_safe=sweep(lo,hi);
      if(!out.arm_safe)out.arm_reason=out.budget_exhausted?"sweep_budget":"collision";
      out.allowed=out.arm_safe && sweep(out.low,lo-1) && sweep(hi+1,out.high);
      out.arm_low=out.allowed?out.low:lo;out.arm_high=out.allowed?out.high:hi;
      // CAN arrival feedback may alternate by one count. Check that small
      // neighboring aperture now instead of weakening the consumer's coverage
      // test. A colliding/over-budget neighbor never revokes the valid required
      // interval and never becomes part of a permit.
      if(out.arm_safe) {
        const int lower=std::max(0,raw-1),upper=std::min(255,raw+1);
        if(lower<out.arm_low && sweep(lower,out.arm_low-1))out.arm_low=lower;
        if(upper>out.arm_high && sweep(out.arm_high+1,upper))out.arm_high=upper;
        if(out.allowed){out.low=out.arm_low;out.high=out.arm_high;}
      }
    }
    out.elapsed_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-started).count();
    return out;
  }
};
