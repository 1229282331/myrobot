// Offline MoveIt/FCL cost comparison, with no ROS node, SDK or device IO.
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <algorithm>
std::string read(const char*p){std::ifstream f(p);if(!f)throw std::runtime_error("missing model");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
 if(argc!=3)return 2;
 auto urdf=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();
 if(!urdf||!srdf->initString(*urdf,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
 planning_scene::PlanningScene scene(model);moveit::core::RobotState state(model);state.setToDefaultValues();
 const auto group=model->getJointModelGroup("me6_group");
 std::vector<double> times;std::string line;int collisions=0;
 collision_detection::CollisionRequest request;request.distance=true;
 while(std::getline(std::cin,line)){
  std::istringstream input(line);std::vector<double>q(6);for(auto&x:q)if(!(input>>x)||!std::isfinite(x))return 4;
  state.setJointGroupPositions(group,q);state.update();
  auto start=std::chrono::steady_clock::now();
  collision_detection::CollisionResult result;scene.checkSelfCollision(request,result,state);
  times.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  collisions+=result.collision;
 }
 if(times.empty())return 5;
 std::sort(times.begin(),times.end());auto percentile=[&](double p){return times.at(std::min(times.size()-1,size_t(p*times.size())));};
 std::cout<<"{\"samples\":"<<times.size()<<",\"collisions\":"<<collisions<<",\"p50_ms\":"<<percentile(.5)<<",\"p95_ms\":"<<percentile(.95)<<",\"p99_ms\":"<<percentile(.99)<<",\"max_ms\":"<<times.back()<<"}"<<std::endl;
}
