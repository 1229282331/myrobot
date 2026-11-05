// Offline policy qualification against independent dense original-ACM FCL.
#include "collision_retreat.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <Eigen/QR>
std::string read(const char* p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing model");return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
 if(argc!=3&&argc!=4)return 2;
 const bool trace=argc==4 && std::string(argv[3])=="trace";
 const bool bounds=argc==4 && std::string(argv[3])=="bounds";
 const bool profile=argc==4 && std::string(argv[3])=="profile";
 auto urdf=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();
 if(!urdf||!srdf->initString(*urdf,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);planning_scene::PlanningScene scene(model);
 auto* group=model->getJointModelGroup("me6_group");CollisionRetreatPolicy policy(model,"me6_group",.01);
 SweepDistanceCertificate surfaces(model,"me6_group",true,false,false);
 SweepDistanceCertificate warm_surfaces(model,"me6_group",true,true);
 size_t warm_checks=0,material_checks=0;double cold_us=0.,warm_us=0.,maximum_error=0.,material_ratio=0.;
 auto compareWarm=[&](const moveit::core::RobotState& pose,const collision_detection::AllowedCollisionMatrix& matrix){
   const auto start=std::chrono::steady_clock::now();
   const auto cold=surfaces.measure(scene,pose,matrix);
   const auto middle=std::chrono::steady_clock::now();
   const auto warm=warm_surfaces.measure(scene,pose,matrix);
   const auto end=std::chrono::steady_clock::now();
   cold_us+=std::chrono::duration<double,std::micro>(middle-start).count();
   warm_us+=std::chrono::duration<double,std::micro>(end-middle).count();
   if(cold.collision!=warm.collision || cold.distances.size()!=warm.distances.size())
     throw std::runtime_error("Warm surface search changed collision/pair coverage");
   for(const auto& [pair,distances]:cold.distances){
     const double error=std::abs(distances[0].distance-warm.distances.at(pair)[0].distance);
     maximum_error=std::max(maximum_error,error);++warm_checks;
     if(error>1e-9)throw std::runtime_error("Warm surface search changed exact distance");
   }
 };
 CollisionRetreatPolicy uncached(model,"me6_group",.01);int case_index=0;
 ArmProximityPolicy proximity(model,"me6_group");auto& acm=scene.getAllowedCollisionMatrixNonConst();acm.setEntry("op_unknown_sweep",true);
 auto distance_acm=proximity.distanceMatrix(acm);
 moveit::core::RobotState state(model),test(model);state.setToDefaultValues();
 std::cout<<std::setprecision(17);std::string line;
 while(std::getline(std::cin,line)){
  std::istringstream in(line);Eigen::VectorXd q(6),target(6);for(auto& v:q)if(!(in>>v))return 4;for(auto& v:target)if(!(in>>v))return 5;
  for(const auto& name:model->getVariableNames())if(name.rfind("op_",0)==0){double v=0;if(!(in>>v))return 6;state.setVariablePosition(name,v);}
  int deadline_ms=2000;if(!(in>>deadline_ms))return 7;
  state.setJointGroupPositions(group,q);state.update();policy.reset();
  if(argc==4&&(std::string(argv[3])=="cartesian"||std::string(argv[3])=="tool")){
    Eigen::MatrixXd jacobian;
    if(!state.getJacobian(group,model->getLinkModel("grasp_tcp"),Eigen::Vector3d::Zero(),jacobian))return 10;
    const auto* parent=group->getJointModels().front()->getParentLinkModel();
    const Eigen::Matrix3d basis=state.getGlobalLinkTransform("controller_base").linear().transpose()*state.getGlobalLinkTransform(parent).linear();
    jacobian.topRows(3)=basis*jacobian.topRows(3).eval();jacobian.bottomRows(3)=basis*jacobian.bottomRows(3).eval();
    if(std::string(argv[3])=="tool"){
      const Eigen::Matrix3d rotation=state.getGlobalLinkTransform("controller_base").linear().transpose()*state.getGlobalLinkTransform("grasp_tcp").linear();
      target.head<3>()=rotation*target.head<3>().eval();target.tail<3>()=rotation*target.tail<3>().eval();
    }
    target=q+jacobian.colPivHouseholderQr().solve(target).eval();
  }
  if(profile){
    const auto distances=surfaces.measure(scene,state,distance_acm);
    std::vector<double> excursion(surfaces.variables.size(),0.);
    for(size_t k=0;k<group->getVariableNames().size();++k){
      auto it=std::find(surfaces.variables.begin(),surfaces.variables.end(),group->getVariableNames()[k]);
      excursion[it-surfaces.variables.begin()]=std::abs(target[k]-q[k]);
    }
    const auto pruned=warm_surfaces.distances(scene,state,distance_acm,excursion,.01);
    const auto narrow=warm_surfaces.refinedMatrix(pruned,distance_acm,excursion,state,.01);
    for(const auto& [pair,entry]:distances.distances){
      collision_detection::AllowedCollision::Type type;
      if(narrow.getAllowedCollision(pair.first,pair.second,type)&&type==collision_detection::AllowedCollision::ALWAYS)continue;
      auto one=distance_acm;for(const auto* link:model->getLinkModels())one.setEntry(link->getName(),true);
      one.setEntry(pair.first,pair.second,false);
      double total=0.;
      for(int i=0;i<11;++i){
        test=state;test.setJointGroupPositions(group,q+(target-q)*(double(i)/10));test.update();
        const auto start=std::chrono::steady_clock::now();warm_surfaces.measure(scene,test,one);
        total+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
      }
      std::cerr<<"pair_profile "<<pair.first<<" "<<pair.second<<" "<<entry[0].distance<<" "<<total/11<<std::endl;
    }
  }
  if(bounds){
    std::vector<double> excursion(surfaces.variables.size(),0.);
    for(size_t k=0;k<group->getVariableNames().size();++k){
      auto it=std::find(surfaces.variables.begin(),surfaces.variables.end(),group->getVariableNames()[k]);
      excursion[it-surfaces.variables.begin()]=std::abs(target[k]-q[k]);
    }
    const auto travel=surfaces.armLinkTravels(group,state,q,target,excursion);
    const auto& collision_links=model->getLinkModelsWithCollisionGeometry();
    if(travel.size()!=collision_links.size())throw std::runtime_error("Missing per-link material bound");
    std::vector<std::array<Eigen::Vector3d,8>> previous(collision_links.size());
    std::vector<std::array<double,8>> lengths(collision_links.size(),std::array<double,8>{});
    for(int sample=0;sample<=200;++sample){
      test=state;test.setJointGroupPositions(group,q+(target-q)*(double(sample)/200));test.update();
      for(size_t k=0;k<collision_links.size();++k){
        const auto* link=collision_links[k];const auto half=.5*link->getShapeExtentsAtOrigin();
        for(int corner=0;corner<8;++corner){
          Eigen::Vector3d local=link->getCenteredBoundingBoxOffset();
          for(int axis=0;axis<3;++axis)local[axis]+=(corner&(1<<axis)?1.:-1.)*half[axis];
          const Eigen::Vector3d point=test.getGlobalLinkTransform(link)*local;
          if(sample)lengths[k][corner]+=(point-previous[k][corner]).norm();
          previous[k][corner]=point;++material_checks;
          if(lengths[k][corner]>travel[k]+1e-9)throw std::runtime_error("Per-link material bound exceeded");
          if(travel[k]>1e-8)material_ratio=std::max(material_ratio,lengths[k][corner]/travel[k]);
        }
      }
    }
    compareWarm(state,distance_acm);
    const auto measured=surfaces.measure(scene,state,distance_acm);
    const auto& links=model->getLinkModelsWithCollisionGeometry();
    for(size_t ai=0;ai<links.size();++ai)for(size_t bi=ai+1;bi<links.size();++bi){
      const auto key=std::minmax(links[ai]->getName(),links[bi]->getName());
      const auto found=measured.distances.find(key);if(found==measured.distances.end())continue;
      const double lower=surfaces.surfaceRootLowerBound(state,ai,bi);
      for(const auto& d:found->second)if(!std::isfinite(lower)||lower<0.||lower>std::max(0.,d.distance)+1e-8){
        std::cerr<<key.first<<" "<<key.second<<" lower "<<lower<<" exact "<<d.distance<<std::endl;
        throw std::runtime_error("Native RSS lower bound exceeded exact triangle distance");
      }
    }
  }
  std::map<std::string,double> hold;
  std::string marker;
  if(in>>marker){
    if(marker!="H")return 8;
    for(const auto& name:model->getVariableNames())if(name.rfind("op_",0)==0){double v;if(!(in>>v))return 9;hold[name]=v;}
  }
  int bounded_checks=0;
  if(!hold.empty()&&case_index<4){
    std::vector<double> uncertainty(surfaces.variables.size(),0.);
    for(const auto& [name,error]:hold){auto at=std::find(surfaces.variables.begin(),surfaces.variables.end(),name);uncertainty[at-surfaces.variables.begin()]=error;}
    for(const auto& key:surfaces.uncertainPairReject(uncertainty,.01).pairs)for(const size_t count:{1,2,4,8}){
      const auto boxes=surfaces.heldErrorBoxes(key,uncertainty,count);
      if(boxes.empty())throw std::runtime_error("Missing held error partition");
      double volume=0.;
      for(const auto& box:boxes){double fraction=1.;
        for(size_t j=0;j<uncertainty.size();++j){
          if(std::abs(box.offset[j])+box.half[j]>uncertainty[j]+1e-14||box.half[j]<0.)
            throw std::runtime_error("Held error cell left original domain");
          if(uncertainty[j]>0.)fraction*=box.half[j]/uncertainty[j];
        }volume+=fraction;
      }
      if(std::abs(volume-1.)>1e-12)throw std::runtime_error("Held error partition lost volume");
      for(size_t a=0;a<boxes.size();++a)for(size_t b=a+1;b<boxes.size();++b){
        bool separate=false;
        for(size_t j=0;j<uncertainty.size();++j)if(uncertainty[j]>0.)
          separate|=std::abs(boxes[a].offset[j]-boxes[b].offset[j])>=boxes[a].half[j]+boxes[b].half[j]-1e-14;
        if(!separate)throw std::runtime_error("Held error cells have overlapping interiors");
      }
    }
  }
  if(argc==4&&std::string(argv[3])=="bounded"){
    std::vector<double> excursion(surfaces.variables.size(),0.);
    for(size_t k=0;k<group->getVariableNames().size();++k){
      auto it=std::find(surfaces.variables.begin(),surfaces.variables.end(),group->getVariableNames()[k]);
      excursion[it-surfaces.variables.begin()]=std::abs(target[k]-q[k]);
    }
    for(const auto& [name,error]:hold){auto it=std::find(surfaces.variables.begin(),surfaces.variables.end(),name);excursion[it-surfaces.variables.begin()]=error;}
    const auto exact=surfaces.measure(scene,state,distance_acm);
    SweepDistanceCertificate::DistanceCache cache;
    const auto bounded=warm_surfaces.distances(scene,state,distance_acm,excursion,.01,&cache);
    for(const auto& [pair,rows]:exact.distances){
      auto found=bounded.distances.find(pair);
      if(found==bounded.distances.end()||found->second.empty())throw std::runtime_error("Bounded search missed pair");
      const double lower=found->second[0].distance,truth=rows[0].distance;
      if(!std::isfinite(lower)||lower>truth+1e-9)throw std::runtime_error("Bounded FCL lower bound exceeded exact distance");
      auto cached=cache.find(pair);
      if(cached!=cache.end()&&std::abs(cached->second-truth)>1e-9)throw std::runtime_error("Finite search radius entered exact cache");
      ++bounded_checks;
    }
    compareWarm(state,distance_acm);
  }
  const bool allowed=policy.assess(scene,state,target,std::chrono::steady_clock::now()+std::chrono::milliseconds(deadline_ms),
      std::chrono::steady_clock::time_point::min(),hold);
  const auto reported=policy.result;
  if(allowed&&!hold.empty()){
    const auto now=std::chrono::steady_clock::now();
    if(!policy.assess(scene,state,target,now+std::chrono::seconds(2),{},hold)||policy.result.identical_path_reuses!=1)
      throw std::runtime_error("Held path did not reuse its exact current proof");
    if(policy.assess(scene,state,target,now-std::chrono::milliseconds(1),{},hold))
      throw std::runtime_error("Held proof bypassed the execution deadline");
    auto invalid=hold;invalid["not_a_model_joint"]=.001;
    if(policy.assess(scene,state,target,now+std::chrono::seconds(2),{},invalid))
      throw std::runtime_error("Held proof survived changed uncertainty");
  }
  if(allowed && hold.empty()){
    // An identical arm endpoint must not reuse a proof for a different or
    // malformed aperture uncertainty request.
    if(policy.assess(scene,state,target,std::chrono::steady_clock::now()+std::chrono::seconds(2),
        std::chrono::steady_clock::time_point::min(),{{"not_a_model_joint",1.}}))
      throw std::runtime_error("Hold uncertainty change reused an unrelated proof");
    policy.reset();
    if(!policy.assess(scene,state,target,std::chrono::steady_clock::now()+std::chrono::seconds(2)))
      throw std::runtime_error("Reset did not restore valid independent proof");
  }
  if(case_index++<12&&deadline_ms>0&&hold.empty()){
    const bool repeated=policy.assess(scene,state,target,std::chrono::steady_clock::now()+std::chrono::seconds(2));
    if(repeated!=allowed || (allowed && policy.result.identical_path_reuses!=1))
      throw std::runtime_error("Identical-path cache changed retreat decision or failed to reuse");
    if(policy.assess(scene,state,target,std::chrono::steady_clock::now()-std::chrono::milliseconds(1)))
      throw std::runtime_error("Expired identical-path cache permitted retreat");
    if(allowed){
      const auto now=std::chrono::steady_clock::now();
      if(!policy.assess(scene,state,target,now-std::chrono::milliseconds(1),now+std::chrono::seconds(2)))
        throw std::runtime_error("Valid completed proof was not reusable before execution deadline");
      if(policy.assess(scene,state,.5*(q+target),now-std::chrono::milliseconds(1),now+std::chrono::seconds(2)))
        throw std::runtime_error("Different target consumed expired computation budget");
      if(policy.assess(scene,state,target,now-std::chrono::milliseconds(1),now-std::chrono::milliseconds(1)))
        throw std::runtime_error("Completed proof bypassed execution deadline");
    }
    for(int scenario=0;scenario<3;++scenario){
      moveit::core::RobotState changed(state);Eigen::VectorXd check_target=.5*(q+target);
      if(scenario==1){Eigen::VectorXd shifted=q;shifted[1]+=.0001;changed.setJointGroupPositions(group,shifted);changed.update();}
      if(scenario==2)scene.getAllowedCollisionMatrixNonConst().setEntry("Link2","base_link",true);
      uncached.reset();
      const bool cached=policy.assess(scene,changed,check_target,std::chrono::steady_clock::now()+std::chrono::seconds(2));
      const bool fresh=uncached.assess(scene,changed,check_target,std::chrono::steady_clock::now()+std::chrono::seconds(2));
      if(cached!=fresh)throw std::runtime_error("Actual-state cache changed retreat decision");
      if(scenario==2)scene.getAllowedCollisionMatrixNonConst().setEntry("Link2","base_link",false);
    }
  }
  int checks=0;bool collision=false,closing=false;
  int held_checks=0;
  if(allowed||trace){
   test=state;collision_detection::CollisionRequest cr;cr.group_name="me6_group";
   collision_detection::DistanceRequest dr;dr.type=collision_detection::DistanceRequestType::SINGLE;dr.group_name="me6_group";dr.enableGroup(model);dr.acm=&distance_acm;
   auto dense_acm=distance_acm;
   const auto complete=surfaces.measure(scene,state,distance_acm);
   // Independent global triangle bound, deliberately looser than the
   // candidate's local interval pruning. No potentially limiting pair is lost.
   const double global_travel=surfaces.armTravel(group,q,target);
   for(const auto& pair:complete.distances)if(pair.second[0].distance>.01+2*global_travel+1e-6)
     dense_acm.setEntry(pair.first.first,pair.first.second,true);
   std::map<std::pair<std::string,std::string>,double> before;
   for(int i=0;i<=200;++i){
    test.setJointGroupPositions(group,q+(target-q)*(double(i)/200));test.update();
    collision_detection::CollisionResult r;scene.checkSelfCollision(cr,r,test,acm);collision|=r.collision;++checks;
    // Full original distance matrix: no candidate broad-phase or interval
    // pruning. Boxes use exact triangulated surfaces to avoid mesh/box GJK
    // feature jitter. Actual collisions above still use original primitives.
    auto distances=surfaces.measure(scene,test,dense_acm);
    if(bounds)compareWarm(test,dense_acm);
    if(trace){
      collision_detection::DistanceResult native;scene.getCollisionEnvUnpadded()->distanceSelf(dr,native,test);
      const auto pair=std::make_pair(std::string("Link3"),std::string("r4_plate"));
      std::cerr<<std::setprecision(17)<<"distance_trace "<<i<<" "<<native.distances.at(pair)[0].distance<<" "<<distances.distances.at(pair)[0].distance<<std::endl;
    }
    for(const auto& pair:distances.distances){double d=std::numeric_limits<double>::infinity();for(const auto& v:pair.second)d=std::min(d,v.distance);
     if(i&&before.count(pair.first)&&d+1e-8<std::min(.01,before[pair.first])){
      if(!closing)std::cerr<<std::setprecision(17)<<"decreasing pair "<<pair.first.first<<" "<<pair.first.second<<" step "<<i<<" before "<<before[pair.first]<<" after "<<d<<std::endl;
      closing=true;
     }before[pair.first]=d;
    }
   }
   if(collision||closing){std::cerr<<"case "<<line<<" collision "<<collision<<" closing "<<closing<<std::endl;throw std::runtime_error("Accepted retreat failed dense independent check");}
  }
  if(allowed&&!hold.empty()&&argc==4&&std::string(argv[3])=="held"){
    // Independent exact-FCL checks at box vertices, individual-axis extremes
    // and deterministic interior errors. These exercise the analytical bound;
    // they are not substituted for its continuous-interval proof.
    uint32_t random=0x30a91u;const double travel=surfaces.armTravel(group,q,target);
    for(size_t sample=0;sample<2*hold.size()+18;++sample){
      auto aperture=state;size_t index=0;
      for(const auto& [name,error]:hold){
        double factor=0.;
        if(sample<2*hold.size())factor=sample/2==index?(sample%2?1.:-1.):0.;
        else if(sample<2*hold.size()+2)factor=sample%2?1.:-1.;
        else {random^=random<<13;random^=random>>17;random^=random<<5;factor=double(random)/UINT32_MAX*2.-1.;}
        aperture.setVariablePosition(name,state.getVariablePosition(name)+error*factor);++index;
      }
      aperture.update();auto dense=distance_acm;
      const auto start=surfaces.measure(scene,aperture,distance_acm);
      for(const auto& pair:start.distances)if(pair.second[0].distance>.01+2*travel+1e-6)
        dense.setEntry(pair.first.first,pair.first.second,true);
      std::map<std::pair<std::string,std::string>,double> previous;
      for(int t=0;t<=20;++t){
        auto pose=aperture;pose.setJointGroupPositions(group,q+(target-q)*(double(t)/20));pose.update();
        collision_detection::CollisionRequest request;request.group_name="me6_group";
        collision_detection::CollisionResult result;scene.checkSelfCollision(request,result,pose,acm);
        if(result.collision)throw std::runtime_error("Held retreat collides at aperture error");
        const auto measured=surfaces.measure(scene,pose,dense);++held_checks;
        for(const auto& [pair,rows]:measured.distances){
          const double distance=rows[0].distance;
          if(t&&previous.count(pair)&&distance+1e-9<std::min(.01,previous[pair])){
            std::cerr<<"held decrease "<<sample<<" "<<pair.first<<" "<<pair.second<<" "<<previous[pair]<<" "<<distance<<std::endl;
            throw std::runtime_error("Held retreat approaches at aperture error");
          }previous[pair]=distance;
        }
      }
    }
  }
  const auto& r=reported;
  std::cout<<"{\"allowed\":"<<allowed<<",\"reason\":\""<<r.reason<<"\",\"samples\":"<<r.samples<<",\"dense_checks\":"<<checks
    <<",\"held_checks\":"<<held_checks<<",\"bounded_checks\":"<<bounded_checks<<",\"actual_collision_us\":"<<r.actual_collision_us<<",\"initial_distances_us\":"<<r.initial_distances_us
    <<",\"sample_queries_us\":"<<r.sample_queries_us<<",\"interval_checks_us\":"<<r.interval_checks_us
    <<",\"elapsed_us\":"<<r.elapsed_us<<",\"travel_m\":"<<r.travel_m<<",\"distance_before\":"<<r.minimum_before_m
    <<",\"distance_after\":"<<r.minimum_after_m<<",\"budget_exhausted\":"<<r.budget_exhausted<<",\"target\":[";
  for(int i=0;i<6;++i)std::cout<<(i?",":"")<<target[i];std::cout<<"]}"<<std::endl;
 }
 if(bounds)std::cerr<<std::setprecision(17)<<"warm_search_validation pairs "<<warm_checks<<" max_error_m "<<maximum_error<<" cold_us "<<cold_us<<" warm_us "<<warm_us<<std::endl;
 if(bounds)std::cerr<<"material_bound_validation checks "<<material_checks<<" maximum_ratio "<<material_ratio<<std::endl;
}
