// Pure policy contracts, intentionally no device or solver transport.
#include "orientation_assist.hpp"
#include <iostream>
int main(){
 Eigen::MatrixXd J=Eigen::MatrixXd::Identity(6,6);
 // Coupled local kinematics demand a wrist correction during X translation.
 J(0,3)=.3;J(3,0)=3.;
 Eigen::VectorXd q=Eigen::VectorXd::Zero(6),lo=Eigen::VectorXd::Constant(6,-3),hi=-lo;
 const auto identity=Eigen::Matrix3d::Identity().eval();
 OrientationAssist a;a.begin(1,identity);Eigen::Matrix3d current=identity;
 double maximum=0,first=0;int outward_clamps=0;
 for(int i=0;i<1600;++i){
  auto v=a.suggest(J,q,lo,hi,current,Eigen::Vector3d(.03,0,0),false,a.speed_limit,100.);
  auto w=v.tail<3>().eval();if(i==0)first=w.norm();
  const Eigen::Matrix3d next=w.norm()? (Eigen::AngleAxisd(w.norm()*a.dt,w.normalized()).toRotationMatrix()*current).eval():current;
  if(!a.accepts(current,next))throw std::runtime_error("Envelope escaped");
  current=next;maximum=std::max(maximum,a.angle(a.anchor,current));
  if(a.reason=="angle_envelope")++outward_clamps;
 }
 if(maximum<a.angle_limit-.01||first>=a.speed_limit*.1||outward_clamps<100)throw std::runtime_error("Envelope not exercised");
 const auto at_edge=current;
 // A solver reset in the same gesture must not silently move the anchor.
 a.stop();a.begin(1,current);
 if(a.angle(a.anchor,identity)>1e-12)throw std::runtime_error("Same epoch reset allowance");
 for(int i=0;i<100;++i){
  auto v=a.suggest(J,q,lo,hi,current,Eigen::Vector3d(-.03,0,0),false,a.speed_limit,100.);
  auto w=v.tail<3>().eval();auto next=(Eigen::AngleAxisd(w.norm()*a.dt,w.normalized()).toRotationMatrix()*current).eval();
  if(!a.accepts(current,next))throw std::runtime_error("Reverse escaped envelope");current=next;
 }
 if(a.angle(a.anchor,current)>=a.angle(a.anchor,at_edge)-.1)throw std::runtime_error("Cannot retreat");
 a.stop();a.begin(2,current);
 if(a.angle(a.anchor,current)>1e-12||a.stage1.norm()!=0||a.stage2.norm()!=0)throw std::runtime_error("Restart retained old goal");
 auto neutral=a.suggest(J,q,lo,hi,current,Eigen::Vector3d::Zero(),true,.01,100.);
 if(neutral.norm()!=0)throw std::runtime_error("Neutral moved");
 const auto outside=(Eigen::AngleAxisd(a.angle_limit+.001,Eigen::Vector3d::UnitZ()).toRotationMatrix()*current).eval();
 if(a.accepts(current,outside))throw std::runtime_error("Accepted invalid final rotation");
 a.stop();a.begin(3,current);
 for(int i=0;i<100;++i){
  auto v=a.suggest(J,q,lo,hi,current,Eigen::Vector3d(.03,0,0),false,a.speed_limit,100.,[](const Eigen::VectorXd& fixed){return fixed.allFinite();});
  if(v.tail<3>().norm()!=0||a.reason!="preserving_retreat")throw std::runtime_error("Interfered with direct retreat");
 }
 std::cout<<"{\"passed\":true,\"steps\":1800,\"max_angle_deg\":"<<maximum*180/a.pi<<",\"edge_steps\":"<<outward_clamps<<",\"first_rate_rad_s\":"<<first<<"}"<<std::endl;
}
