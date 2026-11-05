// Offline check of production certificates against dense original-ACM FCL.
#include "sweep_distance.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
std::string read(const char* p){std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){
 if(argc!=4)return 2;
 auto urdf=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();
 if(!urdf||!srdf->initString(*urdf,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);planning_scene::PlanningScene scene(model);
 SweepDistanceCertificate certificate(model);if(!certificate.valid())return 4;
 auto arm=model->getJointModelGroup("me6_group");
 std::ifstream table(argv[3]);std::vector<std::string> names(8);for(auto& n:names)table>>n;
 std::vector<std::vector<double>> poses(256,std::vector<double>(8));for(auto& row:poses)for(auto& x:row)table>>x;
 auto acm=scene.getAllowedCollisionMatrix();acm.setEntry("op_unknown_sweep",true);
 collision_detection::CollisionRequest req;req.group_name="e6_gripper_geometry";
 moveit::core::RobotState state(model);state.setToDefaultValues();
 std::string line;std::cout<<std::setprecision(17);
 while(std::getline(std::cin,line)) {
  std::istringstream in(line);std::vector<double> q(6),target(6);int raw,lo,hi;
  for(auto& x:q)in>>x;for(auto& x:target)in>>x;in>>raw>>lo>>hi;
  state.setJointGroupPositions(arm,q);for(int i=0;i<8;++i)state.setVariablePosition(names[i],poses[raw][i]);state.update();
  std::vector<double> excursion(certificate.variables.size(),0.);
  for(size_t v=0;v<certificate.variables.size();++v) {
   const auto& n=certificate.variables[v];auto ai=std::find(arm->getVariableNames().begin(),arm->getVariableNames().end(),n);
   if(ai!=arm->getVariableNames().end())excursion[v]=std::abs(target[ai-arm->getVariableNames().begin()]-q[ai-arm->getVariableNames().begin()]);
   auto gi=std::find(names.begin(),names.end(),n);
   if(gi!=names.end())for(int r=lo;r<=hi;++r)excursion[v]=std::max(excursion[v],std::abs(poses[r][gi-names.begin()]-poses[raw][gi-names.begin()]));
  }
  auto started=std::chrono::steady_clock::now();auto distances=certificate.distances(scene,state,acm,excursion);
  bool certified=certificate.certifies(distances,acm,excursion,state);
  auto narrow_acm=certificate.refinedMatrix(distances,acm,excursion,state);
  double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-started).count();
  auto missing=distances;missing.distances.clear();if(certificate.certifies(missing,acm,excursion,state))throw std::runtime_error("Missing distances permitted");
  auto nan=excursion;nan[0]=NAN;if(certificate.certifies(distances,acm,nan,state))throw std::runtime_error("NaN permitted");
  double reach[6]={1,.9,.7,.5,.4,.3},travel=0;for(int i=0;i<6;++i)travel+=reach[i]*std::abs(target[i]-q[i]);
  Eigen::Map<Eigen::VectorXd> qv(q.data(),6),tv(target.data(),6);
  double global_travel=certificate.armTravel(arm,qv,tv);
  double model_travel=certificate.armTravel(arm,state,qv,tv,excursion);
  // Independent material-point check: transform every mesh vertex / box corner
  // through sampled arm motion and compare each step against the derived bound.
  std::vector<Eigen::Vector3d> previous;double max_point_step=0.;
  for(int aperture:std::set<int>{lo,raw,(lo+hi)/2,hi}) {
   for(int i=0;i<8;++i)state.setVariablePosition(names[i],poses[aperture][i]);
   for(int a=0;a<=10;++a) {
   std::vector<double> x(6);for(int i=0;i<6;++i)x[i]=q[i]+(target[i]-q[i])*double(a)/10.;
   state.setJointGroupPositions(arm,x);state.update();std::vector<Eigen::Vector3d> points;
   for(const auto* link:model->getLinkModelsWithCollisionGeometry())for(size_t k=0;k<link->getShapes().size();++k) {
    auto transform=state.getGlobalLinkTransform(link)*link->getCollisionOriginTransforms()[k];
    const auto& shape=link->getShapes()[k];
    if(shape->type==shapes::MESH) {
     const auto* mesh=static_cast<const shapes::Mesh*>(shape.get());
     for(unsigned v=0;v<mesh->vertex_count;++v)points.push_back(transform*Eigen::Vector3d(mesh->vertices+3*v));
    } else if(shape->type==shapes::BOX) {
     const auto* box=static_cast<const shapes::Box*>(shape.get());
     for(int i:{-1,1})for(int j:{-1,1})for(int l:{-1,1})points.push_back(transform*Eigen::Vector3d(i*box->size[0]/2,j*box->size[1]/2,l*box->size[2]/2));
    }
   }
   if(a)for(size_t i=0;i<points.size();++i)max_point_step=std::max(max_point_step,(points[i]-previous[i]).norm());
   previous=std::move(points);
   }
  }
  if(max_point_step>model_travel/10.+1e-10)throw std::runtime_error("Material point exceeds arm travel bound");
  int steps=std::max(1,int(std::ceil(travel/.001))),checks=0;bool collision=false;
  // Production rejects these oversized intervals before any collision query.
  // Validate their point bound above; don't spend millions of irrelevant FCL
  // calls qualifying an interval that cannot receive a production permit.
  const bool oversized=model_travel>.04;
  const auto prepared=certificate.preparePairs(scene,req,state,narrow_acm);
  for(int a=0;a<=steps&&!collision&&!oversized;++a) {
   for(int r=lo;r<=hi&&!collision;++r) {
   std::vector<double> x(6);for(int i=0;i<6;++i)x[i]=q[i]+(target[i]-q[i])*double(a)/steps;
   state.setJointGroupPositions(arm,x);for(int i=0;i<8;++i)state.setVariablePosition(names[i],poses[r][i]);state.update();
   collision_detection::CollisionResult result;scene.checkSelfCollision(req,result,state,acm);++checks;collision=result.collision;
   collision_detection::CollisionResult pruned;
   if(!certificate.checkPairs(prepared,state,pruned))scene.checkSelfCollision(req,pruned,state,narrow_acm);
   if(pruned.collision!=result.collision)throw std::runtime_error("Pruning changed dense collision decision");
   }
  }
  if(certified&&collision)throw std::runtime_error("Certificate missed a dense sweep collision");
  std::cout<<"{\"certified\":"<<certified<<",\"oversized_rejected\":"<<oversized<<",\"dense_collision\":"<<collision<<",\"checks\":"<<checks<<",\"certificate_us\":"<<us<<",\"model_travel_m\":"<<model_travel<<",\"global_travel_m\":"<<global_travel<<",\"legacy_travel_m\":"<<travel<<"}"<<std::endl;
 }
}
