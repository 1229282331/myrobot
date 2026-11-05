// Read-only MoveIt/FCL FK and collision sampling. No ROS node or device IO.
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

std::string read(const char* p){std::ifstream f(p);if(!f)throw std::runtime_error("missing model");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
  if(argc!=3&&argc!=4)return 2;
  auto urdf=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();
  if(!urdf||!srdf->initString(*urdf,read(argv[2])))return 3;
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  planning_scene::PlanningScene scene(model);moveit::core::RobotState state(model);state.setToDefaultValues();
  const auto group=model->getJointModelGroup("me6_group");
  collision_detection::CollisionRequest request;request.contacts=true;request.max_contacts=32;
  if(argc==4)request.group_name=argv[3];
  std::cout<<std::setprecision(17);std::string line;
  while(std::getline(std::cin,line)){
    std::istringstream input(line);std::vector<double> q(6);for(auto&x:q)if(!(input>>x)||!std::isfinite(x))return 4;
    state.setJointGroupPositions(group,q);
    auto acm=scene.getAllowedCollisionMatrix();
    int known=0;input>>known;
    for(auto& name:model->getVariableNames())if(name.rfind("op_",0)==0){
      double value=0;if(known&&!(input>>value))return 5;
      state.setVariablePosition(name,value);
    }
    if(known)acm.setEntry("op_unknown_sweep",true);
    else for(auto* link:model->getLinkModelsWithCollisionGeometry())if(link->getName().rfind("op_hand_",0)==0)acm.setEntry(link->getName(),true);
    state.update();
    collision_detection::CollisionResult result;scene.checkSelfCollision(request,result,state,acm);
    auto t=state.getGlobalLinkTransform("controller_base").inverse()*state.getGlobalLinkTransform("grasp_tcp");
    Eigen::Quaterniond r(t.rotation());
    std::cout<<"{\"collision\":"<<(result.collision?"true":"false")<<",\"position_m\":["<<t.translation().x()<<","<<t.translation().y()<<","<<t.translation().z()<<"],\"quaternion_xyzw\":["<<r.x()<<","<<r.y()<<","<<r.z()<<","<<r.w()<<"],\"contacts\":[";
    bool comma=false;for(const auto& pair:result.contacts){if(comma)std::cout<<",";comma=true;std::cout<<"[\""<<pair.first.first<<"\",\""<<pair.first.second<<"\"]";}
    std::cout<<"]}"<<std::endl;
  }
}
