// Offline equality check: exact all-shape enumeration vs RSS branch-and-bound.
#include "collision_retreat.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
std::string read(const char* p){std::ifstream f(p);if(!f)throw std::runtime_error("missing input");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
 if(argc!=3)return 2;
 auto u=urdf::parseURDF(read(argv[1]));auto s=std::make_shared<srdf::Model>();if(!u||!s->initString(*u,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(u,s);planning_scene::PlanningScene scene(model);
 auto* group=model->getJointModelGroup("me6_group");ArmProximityPolicy proximity(model,"me6_group");
 scene.getAllowedCollisionMatrixNonConst().setEntry("op_unknown_sweep",true);
 auto acm=proximity.distanceMatrix(scene.getAllowedCollisionMatrix());
 SweepDistanceCertificate exact(model,"me6_group",true,false,false),optimized(model,"me6_group",true,true,true);
 RetreatDistanceWorkers workers;
 SweepDistanceCertificate::Parallel parallel=[&](size_t n,std::function<void(size_t)> f){workers.run(n,std::move(f));};
 moveit::core::RobotState state(model);state.setToDefaultValues();
 std::string line;size_t samples=0,pairs=0;double error=0.,cold_us=0.,warm_us=0.;
 while(std::getline(std::cin,line)){
  std::istringstream in(line);std::vector<double> q(6);for(auto&v:q)if(!(in>>v))return 4;
  state.setJointGroupPositions(group,q);
  for(const auto& name:model->getVariableNames())if(name.rfind("op_",0)==0){double v;if(!(in>>v))return 5;state.setVariablePosition(name,v);}
  state.update();auto start=std::chrono::steady_clock::now();auto a=exact.measure(scene,state,acm);
  auto middle=std::chrono::steady_clock::now();auto b=optimized.measure(scene,state,acm,nullptr,nullptr,0.,nullptr,parallel);auto end=std::chrono::steady_clock::now();
  cold_us+=std::chrono::duration<double,std::micro>(middle-start).count();warm_us+=std::chrono::duration<double,std::micro>(end-middle).count();
  if(a.collision!=b.collision||a.distances.size()!=b.distances.size())return 6;
  for(const auto& [key,values]:a.distances){
   double d=std::abs(values[0].distance-b.distances.at(key)[0].distance);error=std::max(error,d);++pairs;
   if(!std::isfinite(d)||d>1e-9){std::cerr<<samples<<' '<<key.first<<' '<<key.second<<' '<<d<<std::endl;return 7;}
  }
  ++samples;
 }
 std::cout<<std::setprecision(17)<<"{\"passed\":true,\"samples\":"<<samples<<",\"pairs\":"<<pairs<<",\"max_error_m\":"<<error
  <<",\"exact_us\":"<<cold_us<<",\"optimized_us\":"<<warm_us<<"}"<<std::endl;
}
