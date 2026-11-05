// Offline only: exact production recovery policy, no device IO.
#include "margin_recovery.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
std::string read(const char*p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing input");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
 if(argc!=4)return 2;
 auto u=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();if(!u||!srdf->initString(*u,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);planning_scene::PlanningScene scene(model);
 MarginRecovery recovery(model);auto*group=model->getJointModelGroup("me6_group");
 std::vector<std::string> names;for(const auto&n:model->getVariableNames())if(n.rfind("op_",0)==0)names.push_back(n);
 std::vector<double> table(256*names.size());std::istringstream poses(read(argv[3]));for(auto&v:table)if(!(poses>>v))return 4;
 auto aperture=[&](moveit::core::RobotState&s,int raw){for(size_t k=0;k<names.size();++k)s.setVariablePosition(names[k],table[raw*names.size()+k]);s.update();};
 std::cout<<std::setprecision(17);std::string line;
 while(std::getline(std::cin,line)){
  std::istringstream in(line);Eigen::VectorXd q(6),end(6);int low,high;for(auto&v:q)if(!(in>>v))return 5;for(auto&v:end)if(!(in>>v))return 6;if(!(in>>low>>high))return 7;
  moveit::core::RobotState state(model);state.setToDefaultValues();state.setJointGroupPositions(group,q);aperture(state,low);
  auto start=std::chrono::steady_clock::now();bool ok=recovery.assess(scene,state,end,low,high,aperture,start+std::chrono::seconds(10));
  auto&r=recovery.result;
  std::cout<<"{\"verified\":"<<(ok?"true":"false")<<",\"reason\":\""<<r.reason<<"\",\"pairs\":"<<r.pairs<<",\"source_clearance_m\":"<<(std::isfinite(r.source_clearance_m)?r.source_clearance_m:-1)<<",\"outward_derivative_m_rad\":"<<(std::isfinite(r.outward_derivative_m_rad)?r.outward_derivative_m_rad:-1)<<",\"elapsed_us\":"<<r.elapsed_us<<"}"<<std::endl;
 }
}
