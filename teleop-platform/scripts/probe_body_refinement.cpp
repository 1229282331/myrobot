// Read-only independent FCL collision comparison of refined and source meshes.
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
std::string read(const char* p){std::ifstream f(p);if(!f)throw std::runtime_error("missing input");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
 if(argc!=4)return 2;
 struct Model {moveit::core::RobotModelPtr model;std::shared_ptr<planning_scene::PlanningScene> scene;std::shared_ptr<moveit::core::RobotState> state;};
 std::vector<Model> models;
 for(int i=1;i<=2;++i){
  auto u=urdf::parseURDF(read(argv[i]));auto s=std::make_shared<srdf::Model>();if(!u||!s->initString(*u,read(argv[3])))return 3;
  auto m=std::make_shared<moveit::core::RobotModel>(u,s);auto scene=std::make_shared<planning_scene::PlanningScene>(m);
  scene->getAllowedCollisionMatrixNonConst().setEntry("op_unknown_sweep",true);
  auto state=std::make_shared<moveit::core::RobotState>(m);state->setToDefaultValues();models.push_back({m,scene,state});
 }
 std::string line;size_t n=0,reference_collisions=0,candidate_collisions=0;double max_us=0;
 while(std::getline(std::cin,line)){
  std::istringstream in(line);std::vector<double> q(6),g(8);for(auto&v:q)if(!(in>>v))return 4;for(auto&v:g)if(!(in>>v))return 5;
  bool collisions[2]={};int index=0;
  for(auto& m:models){
   m.state->setJointGroupPositions(m.model->getJointModelGroup("me6_group"),q);int j=0;
   for(const auto& name:m.model->getVariableNames())if(name.rfind("op_",0)==0)m.state->setVariablePosition(name,g.at(j++));
   m.state->update();collision_detection::CollisionRequest req;collision_detection::CollisionResult result;
   auto start=std::chrono::steady_clock::now();m.scene->checkSelfCollision(req,result,*m.state);
   double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
   if(index==0)max_us=std::max(max_us,us);collisions[index++]=result.collision;
  }
  ++n;candidate_collisions+=collisions[0];reference_collisions+=collisions[1];
  if(collisions[1]&&!collisions[0]){std::cerr<<"SOURCE_COLLISION_NOT_COVERED "<<n<<std::endl;return 6;}
 }
 std::cout<<"{\"passed\":true,\"samples\":"<<n<<",\"candidate_collisions\":"<<candidate_collisions
 <<",\"source_collisions\":"<<reference_collisions<<",\"candidate_max_us_pc\":"<<max_us<<"}"<<std::endl;
}
