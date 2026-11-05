// Offline qualification of the production distance policy; no device IO.
#include "arm_proximity.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
std::string read(const char* p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing model");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char** argv){
 if(argc!=3)return 2;
 auto urdf=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();
 if(!urdf||!srdf->initString(*urdf,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);planning_scene::PlanningScene scene(model);
 moveit::core::RobotState state(model);state.setToDefaultValues();auto* group=model->getJointModelGroup("me6_group");
 ArmProximityPolicy policy(model,"me6_group");
 std::cerr<<"INVARIANT_PAIRS\n";for(const auto& p:policy.invariantPairs())std::cerr<<p.first<<" "<<p.second<<"\n";
 collision_detection::CollisionRequest req;req.group_name="me6_group";req.distance=true;req.contacts=true;req.max_contacts=64;
 std::cout<<std::setprecision(17);std::string line;
 while(std::getline(std::cin,line)){
  std::istringstream in(line);std::vector<double> q(6);for(auto& v:q)if(!(in>>v))return 4;
  state.setJointGroupPositions(group,q);int known;in>>known;
  for(const auto& name:model->getVariableNames())if(name.rfind("op_",0)==0){double v=0;if(known&&!(in>>v))return 5;state.setVariablePosition(name,v);}
  state.update();auto acm=scene.getAllowedCollisionMatrix();
  if(known)acm.setEntry("op_unknown_sweep",true);
  else for(auto* l:model->getLinkModelsWithCollisionGeometry())if(l->getName().rfind("op_hand_",0)==0)acm.setEntry(l->getName(),true);
  collision_detection::CollisionResult original,filtered;scene.getCollisionEnvUnpadded()->checkSelfCollision(req,original,state,acm);
  policy.check(scene.getCollisionEnvUnpadded(),req,filtered,state,acm);
  if(original.collision!=filtered.collision)throw std::runtime_error("Actual collision changed");
  const auto& nearest=filtered.distance_result.minimum_distance;
  std::cout<<"{\"collision\":"<<original.collision<<",\"original_distance\":"<<original.distance<<",\"distance\":"<<filtered.distance<<",\"nearest\":[\""<<nearest.link_names[0]<<"\",\""<<nearest.link_names[1]<<"\"],\"contacts\":[";
  bool comma=false;for(const auto& p:original.contacts){if(comma)std::cout<<",";comma=true;std::cout<<"[\""<<p.first.first<<"\",\""<<p.first.second<<"\"]";}
  std::cout<<"]}"<<std::endl;
 }
}
