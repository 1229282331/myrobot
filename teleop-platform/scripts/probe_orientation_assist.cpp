// Independent FK/Jacobian and bounded helper checks; no hardware transport.
#include "orientation_assist.hpp"
#include <moveit/robot_state/robot_state.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
std::string read(const char* p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing model");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
 if(argc!=3)return 2;
 auto urdf=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();
 if(!urdf||!srdf->initString(*urdf,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
 auto* group=model->getJointModelGroup("me6_group");moveit::core::RobotState s(model),probe(model);
 s.setToDefaultValues();Eigen::VectorXd lower(6),upper(6);
 for(int k=0;k<6;++k){const auto& b=model->getVariableBounds("joint"+std::to_string(k+1));lower[k]=b.min_position_+.0872664626;upper[k]=b.max_position_-.0872664626;}
 auto pose=[&](const moveit::core::RobotState& x)->Eigen::Isometry3d{return x.getGlobalLinkTransform("controller_base").inverse()*x.getGlobalLinkTransform("grasp_tcp");};
 std::cout<<std::setprecision(17);std::string line;int n=0;
 while(std::getline(std::cin,line)){
  std::istringstream in(line);Eigen::VectorXd q(6);Eigen::Vector3d linear;int tool;
  for(auto& v:q)if(!(in>>v))return 4;for(auto& v:linear)if(!(in>>v))return 5;if(!(in>>tool))return 6;
  s.setJointGroupPositions(group,q);s.update();const auto start=pose(s);
  Eigen::MatrixXd J;s.getJacobian(group,model->getLinkModel("grasp_tcp"),Eigen::Vector3d::Zero(),J);
  const auto parent=group->getJointModels().front()->getParentLinkModel();
  const Eigen::Matrix3d basis=s.getGlobalLinkTransform("controller_base").rotation().transpose()*s.getGlobalLinkTransform(parent).rotation();
  J.topRows(3)=basis*J.topRows(3).eval();J.bottomRows(3)=basis*J.bottomRows(3).eval();
  double error=0;
  for(int k=0;k<6;++k){
   probe=s;Eigen::VectorXd p=q;p[k]+=1e-7;probe.setJointGroupPositions(group,p);probe.update();const auto next=pose(probe);
   const Eigen::AngleAxisd rot(next.rotation()*start.rotation().transpose());Eigen::Matrix<double,6,1> measured;
   measured<<((next.translation()-start.translation())/1e-7),rot.axis()*rot.angle()/1e-7;
   error=std::max(error,(measured-J.col(k)).cwiseAbs().maxCoeff());
  }
  if(error>2e-6)throw std::runtime_error("TCP Jacobian frame mismatch");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J,Eigen::ComputeFullU|Eigen::ComputeFullV);auto sv=svd.singularValues();
  const auto native_condition=[&](moveit::core::RobotState& st){
   Eigen::JacobiSVD<Eigen::MatrixXd> raw(st.getJacobian(group));const auto values=raw.singularValues();return values[0]/values[5];
  };
  const double condition=native_condition(s);OrientationAssist a;a.begin(++n,start.rotation());
  const auto strict_escape=[&](const Eigen::VectorXd& fixed){
   if(condition<30||fixed.norm()<1e-12)return false;
   probe=s;const Eigen::VectorXd next_q=q+fixed*std::min(.01,1e-4/fixed.norm());probe.setJointGroupPositions(group,next_q);probe.update();
   return native_condition(probe)<condition-1e-8;
  };
  Eigen::Matrix<double,6,1> v;double maximum=0;
  for(int k=0;k<100;++k){v=a.suggest(J,q,lower,upper,start.rotation(),linear,tool,.03490658503988659,condition,strict_escape);maximum=std::max(maximum,v.tail<3>().norm());}
  const bool preserved_retreat=a.reason=="preserving_retreat";
  const Eigen::Vector3d expected=tool?start.rotation()*linear:linear;
  if((v.head<3>()-expected).norm()>1e-12||maximum>.03490658503988659+1e-12)throw std::runtime_error("Helper changed translation/speed");
  const Eigen::VectorXd fixed=svd.solve((Eigen::Matrix<double,6,1>()<<expected,Eigen::Vector3d::Zero()).finished());
  const Eigen::VectorXd assisted=svd.solve(v);
  // An exact reset cannot remember angular momentum or return to old pose.
  const auto acquired=(Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitX()).toRotationMatrix()*start.rotation()).eval();
  a.stop();a.begin(++n,acquired);if(a.stage1.norm()!=0||a.stage2.norm()!=0||OrientationAssist::angle(a.anchor,acquired)>1e-12)throw std::runtime_error("Stale assisted state");
  a.suggest(J,q,lower,upper,acquired,Eigen::Vector3d::Zero(),tool,.03,condition);
  if(a.twist.norm()!=0)throw std::runtime_error("Neutral generated motion");
  std::cout<<"{\"jacobian_error\":"<<error<<",\"condition\":"<<condition<<",\"preserved_retreat\":"<<(preserved_retreat?"true":"false")<<",\"fixed_effort\":"<<fixed.norm()<<",\"assisted_effort\":"<<assisted.norm()<<",\"angular_speed\":"<<maximum<<",\"twist\":[";
  for(int k=0;k<6;++k){if(k)std::cout<<",";std::cout<<v[k];}std::cout<<"]}"<<std::endl;
 }
}
