#include "orientation_assist.hpp"

Eigen::Matrix<double,6,1> OrientationAssist::suggest(const Eigen::MatrixXd& J,
      const Eigen::VectorXd& q,const Eigen::VectorXd& lower,const Eigen::VectorXd& upper,
      const Eigen::Matrix3d& current,const Eigen::Vector3d& linear,bool tool,double max_rate,
      double condition,const std::function<bool(const Eigen::VectorXd&)>& strict_escape) {
    twist.setZero();twist.head<3>()=tool?anchor*linear:linear;
    requested_limit=std::min(speed_limit,max_rate);
    deviation=angle(anchor,current);pressure=0.;reason="holding_orientation";
    if(!initialized||J.rows()!=6||J.cols()!=6||!J.allFinite()||!q.allFinite()||
       !std::isfinite(max_rate)||max_rate<=0||max_rate>speed_limit+1e-12)
      throw std::runtime_error("Invalid orientation assistance input");
    if(linear.norm()<1e-12){stop();return twist;}
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J,Eigen::ComputeFullU|Eigen::ComputeFullV);
    if(svd.singularValues().minCoeff()<=1e-10){reason="singular";stage1.setZero();stage2.setZero();return twist;}
    const Eigen::MatrixXd inverse=svd.solve(Eigen::MatrixXd::Identity(6,6));
    const Eigen::VectorXd fixed=inverse.leftCols(3)*twist.head<3>();
    const Eigen::MatrixXd angular=inverse.rightCols(3);
    Eigen::VectorXd weights=Eigen::VectorXd::Ones(6);
    for(int i=0;i<6;++i){
      const double margin=std::min(q[i]-lower[i],upper[i]-q[i]);
      const double near=std::clamp((20*pi/180-margin)/(15*pi/180),0.,1.);
      weights[i]+=8*near*near;pressure=std::max(pressure,near);
    }
    pressure=std::max(pressure,std::clamp((condition-20.)/25.,0.,1.));
    pressure=std::max(pressure,std::clamp((fixed.cwiseAbs().maxCoeff()-.12)/.25,0.,1.));
    pressure=pressure*pressure*(3-2*pressure);
    const Eigen::Matrix3d h=angular.transpose()*weights.asDiagonal()*angular+4*Eigen::Matrix3d::Identity();
    Eigen::Vector3d desired=-pressure*h.ldlt().solve(angular.transpose()*weights.asDiagonal()*fixed);
    if(!desired.allFinite())throw std::runtime_error("Nonfinite assisted rotation");
    // Do not trade a verified direct retreat for lower joint-rate effort.
    // Decay the suggestion through the same soft-start filter on reversal.
    const bool retreat=strict_escape && strict_escape(fixed);
    if(retreat)desired.setZero();
    if(desired.norm()>requested_limit)desired*=requested_limit/desired.norm();
    // Same two-stage, exact-ZOH soft start as operator input (tau=0.2 s).
    const double a=std::exp(-dt/.2),b=dt/.2*a;
    const Eigen::Vector3d old=stage1;
    stage1=a*stage1+(1-a)*desired;
    stage2=a*stage2+b*old+(1-a-b)*desired;
    Eigen::Vector3d w=stage2;
    if(w.norm()>requested_limit)w*=requested_limit/w.norm();
    // Slow outward angular motion near the per-gesture envelope. Never add
    // a restoring rotation: reaching the envelope holds the acquired pose.
    const Eigen::AngleAxisd offset(current*anchor.transpose());
    if(deviation>1e-9){
      const double outward=w.dot(offset.axis());
      if(outward>0)w-=offset.axis()*outward*(1-std::clamp((angle_limit-deviation)/(2*pi/180),0.,1.));
    }
    for(int i=0;i<20 && w.norm()>0 && angle(anchor,
        (Eigen::AngleAxisd(w.norm()*dt,w.normalized()).toRotationMatrix()*current).eval())>angle_limit-1e-6;++i)w*=.5;
    if(deviation>=angle_limit-1e-6 && w.dot(offset.axis())>0)w.setZero();
    twist.tail<3>()=w;
    reason=deviation>angle_limit-2*pi/180?"angle_envelope":retreat?"preserving_retreat":w.norm()>1e-7?"assisting":"holding_orientation";
    return twist;
  }
