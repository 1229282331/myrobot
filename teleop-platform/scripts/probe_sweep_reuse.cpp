// Probe-only regression for exact per-sweep reuse. No device IO.
#include "dynamic_gripper.hpp"
#include "baseline_sweep_distance.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <limits>
#include <sstream>

static void require(bool test,const char* message){
  if(!test)throw std::runtime_error(message);
}
static bool sameMatrix(const collision_detection::AllowedCollisionMatrix& a,
                       const collision_detection::AllowedCollisionMatrix& b){
  moveit_msgs::msg::AllowedCollisionMatrix x,y;a.getMessage(x);b.getMessage(y);return x==y;
}
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("e6_sweep_reuse_probe",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto monitor=std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node,"robot_description");
  auto model=monitor->getRobotModel();auto* group=model->getJointModelGroup("me6_group");
  DynamicGripper gripper(node,monitor);
  SweepDistanceCertificate current(model,"me6_group",true,true),other(model,"me6_group",true,true);
  BaselineSweepDistanceCertificate baseline(model,"me6_group",true,true);
  SweepDistanceCertificate::ClearPairCache cache(current);
  sensor_msgs::msg::JointState feedback;gripper.feedback(feedback,30);
  planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
  const auto original=scene->getAllowedCollisionMatrix();
  moveit::core::RobotState state(model);state.setToDefaultValues();
  collision_detection::CollisionRequest req;req.group_name="me6_group";
  size_t cases=0,checks=0,collisions=0,proofs=0,conditional_calls=0;
  std::string line;bool conditional_tested=false;
  while(std::getline(std::cin,line)){
    std::istringstream in(line);Eigen::VectorXd q(6),target(6);int nominal;
    for(auto& x:q)require(bool(in>>x),"Invalid input");
    for(auto& x:target)require(bool(in>>x),"Invalid target");
    require(bool(in>>nominal),"Invalid aperture");
    for(const auto& pose:std::vector<Eigen::VectorXd>{q,target,q}){
      state.setJointGroupPositions(group,pose);state.update();
      for(int raw:{0,30,31,128,255,30}){
        gripper.position(state,raw);
        auto prepared=current.preparePairs(*scene.operator->(),req,state,original);
        auto old=baseline.preparePairs(*scene.operator->(),req,state,original);
        collision_detection::CollisionResult reused,plain,oracle;
        require(current.checkPairs(prepared,state,reused,&cache),"Prepared pair fallback");
        require(baseline.checkPairs(old,state,plain),"Baseline fallback");
        auto oracle_req=req;oracle_req.contacts=true;oracle_req.max_contacts=100;
        scene->checkSelfCollision(oracle_req,oracle,state,original);
        require(reused.collision==plain.collision&&plain.collision==oracle.collision,
                "Cached collision decision differs");
        ++checks;collisions+=oracle.collision;
        // A cache from another immutable geometry owner must be ignored.
        const auto before=cache.hits();
        auto different=other.preparePairs(*scene.operator->(),req,state,original);
        collision_detection::CollisionResult foreign;
        require(other.checkPairs(different,state,foreign,&cache),"Foreign fallback");
        require(cache.hits()==before&&foreign.collision==oracle.collision,"Foreign cache used");
        ++checks;
        if(!conditional_tested&&(oracle.contacts.count({"base_link","Link3"})||
                                oracle.contacts.count({"Link3","base_link"}))){
          // Isolate the known mesh contact; a changing conditional callback
          // must still execute at an identical pose, despite the shared cache.
          auto matrix=original;
          const auto& all_links=model->getLinkModelNames();
          matrix.setEntry(all_links,all_links,true);
          bool allow=true;
          collision_detection::DecideContactFn fn=[&](collision_detection::Contact&){
            ++conditional_calls;return allow;
          };
          matrix.setEntry("base_link","Link3",fn);
          auto conditional=current.preparePairs(*scene.operator->(),req,state,matrix);
          require(conditional.indices.size()==1,"Conditional fixture must isolate exactly one pair");
          const auto hits=cache.hits();
          collision_detection::CollisionResult accepted,rejected;
          current.checkPairs(conditional,state,accepted,&cache);
          const auto calls=conditional_calls;allow=false;
          current.checkPairs(conditional,state,rejected,&cache);
          require(!accepted.collision&&rejected.collision&&calls>0&&conditional_calls>calls,
                  "Conditional contact was memoized");
          require(cache.hits()==hits,"Conditional cache hit");
          conditional_tested=true;checks+=2;
        }
      }
    }
    state.setJointGroupPositions(group,q);state.update();gripper.position(state,30);
    std::vector<double> excursion(current.variables.size(),0.);
    auto distances=current.distances(*scene.operator->(),state,original,excursion);
    for(int scenario=0;scenario<8;++scenario){
      auto e=excursion;auto d=distances;auto matrix=original;
      if(scenario==1)e.clear();
      if(scenario==2)e[0]=-1.;
      if(scenario==3)e[0]=std::numeric_limits<double>::quiet_NaN();
      if(scenario==4)d.collision=true;
      if(scenario==5)d.distances.clear();
      if(scenario==6){
        collision_detection::DecideContactFn fn=[](collision_detection::Contact&){return true;};
        matrix.setEntry("base_link","Link3",fn);
      }
      if(scenario==7)for(auto& angle:e)angle=.02;
      collision_detection::AllowedCollisionMatrix refined;
      const bool complete=current.certifiesOrRefines(d,matrix,e,state,refined);
      auto previous=baseline.refinedMatrix(d,matrix,e,state);
      if(!complete)require(sameMatrix(refined,previous),"Refined ACM changed");
      require(complete==baseline.certifies(d,matrix,e,state),"Complete proof changed");
      ++proofs;
    }
    ++cases;
  }
  require(cases>0&&collisions>0&&cache.hits()>0&&conditional_tested,"Insufficient regression coverage");
  SweepDistanceCertificate::ClearPairCache fresh(current);
  require(fresh.hits()==0&&fresh.size()==0,"New sweep inherited old cache");
  std::cout<<"{\"passed\":true,\"physical_io\":false,\"cases\":"<<cases
           <<",\"collision_checks\":"<<checks<<",\"collisions\":"<<collisions
           <<",\"proof_checks\":"<<proofs<<",\"cache_hits\":"<<cache.hits()
           <<",\"cache_entries\":"<<cache.size()<<",\"conditional_calls\":"<<conditional_calls<<"}\n";
  rclcpp::shutdown();
}
