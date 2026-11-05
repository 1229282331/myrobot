// SPDX-License-Identifier: BSD-3-Clause
#include "margin_recovery.hpp"
#include <moveit/robot_model/revolute_joint_model.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <set>

namespace {
using Points=std::vector<Eigen::Vector3d>;
Points vertices(const shapes::ShapeConstPtr& shape) {
  Points p;
  if(shape->type==shapes::BOX){
    const auto* b=static_cast<const shapes::Box*>(shape.get());
    for(int k=0;k<8;++k)p.emplace_back((k&1?1:-1)*b->size[0]/2,(k&2?1:-1)*b->size[1]/2,(k&4?1:-1)*b->size[2]/2);
  }else if(shape->type==shapes::MESH){
    const auto* m=static_cast<const shapes::Mesh*>(shape.get());
    for(unsigned k=0;k<m->vertex_count;++k)p.emplace_back(m->vertices+3*k);
  }
  std::sort(p.begin(),p.end(),[](const auto&a,const auto&b){for(int k=0;k<3;++k)if(a[k]!=b[k])return a[k]<b[k];return false;});
  p.erase(std::unique(p.begin(),p.end(),[](const auto&a,const auto&b){return a==b;}),p.end());return p;
}
Points transform(const Points& p,const Eigen::Isometry3d& t){Points q;q.reserve(p.size());for(const auto&v:p)q.push_back(t*v);return q;}
double gap(const Points&a,const Points&b,const Eigen::Vector3d&n){
  double hi=-INFINITY,lo=INFINITY;for(const auto&v:a)hi=std::max(hi,n.dot(v));for(const auto&v:b)lo=std::min(lo,n.dot(v));return lo-hi;
}
bool descendant(const moveit::core::LinkModel* link,const moveit::core::JointModel* joint){
  for(auto*p=link;p;p=p->getParentLinkModel())if(p->getParentJointModel()==joint)return true;return false;
}
}

MarginRecovery::MarginRecovery(moveit::core::RobotModelConstPtr model):model_(model),arm_(model->getJointModelGroup("me6_group")),certificate_(model,"me6_group",true,true){
  for(const auto*link:model_->getLinkModelsWithCollisionGeometry()){
    auto& parts=parts_[link->getName()];
    for(const auto&shape:link->getShapes()){
      Part part{vertices(shape),shape->type==shapes::BOX,{}};
      if(shape->type==shapes::MESH){
        const auto*m=static_cast<const shapes::Mesh*>(shape.get());
        std::set<std::array<int,3>> seen_normals;
        for(unsigned k=0;k<m->triangle_count;++k){
          const Eigen::Vector3d a(m->vertices+3*m->triangles[3*k]),b(m->vertices+3*m->triangles[3*k+1]),c(m->vertices+3*m->triangles[3*k+2]);
          Eigen::Vector3d n=(b-a).cross(c-a);
          if(n.norm()>1e-12){n.normalize();std::array<int,3> key;
            for(int i=0;i<3;++i)key[i]=int(std::round(n[i]*10000));
            if(seen_normals.insert(key).second)part.normals.push_back(n);
          }
        }
      }
      parts.push_back(std::move(part));
    }
    // These are the same pinned official rigid finger meshes shown by URDF.
    // Their convex support encloses every source vertex/triangle. Source CAD
    // is an ADDITIONAL clearance requirement, never an ordinary collision ACM.
    if(link->getName().rfind("op_hand_",0)==0&&!link->getVisualMeshFilename().empty()){
      shapes::ShapeConstPtr mesh(shapes::createMeshFromResource(link->getVisualMeshFilename(),link->getVisualMeshScale()));
      if(mesh)sources_[link->getName()]=transform(vertices(mesh),link->getVisualMeshOrigin());
    }
  }
}

bool MarginRecovery::assess(const planning_scene::PlanningScene& scene,const moveit::core::RobotState& actual,
    const Eigen::VectorXd& target,int low,int high,
    const std::function<void(moveit::core::RobotState&,int)>& aperture,
    std::chrono::steady_clock::time_point deadline){
  using Clock=std::chrono::steady_clock;const auto started=Clock::now();result={};result.attempted=true;
  auto finish=[&](bool ok,const std::string&why){result.verified=ok;result.reason=why;result.elapsed_us=std::chrono::duration<double,std::micro>(Clock::now()-started).count();return ok;};
  auto expired=[&]{return Clock::now()>=deadline;};
  if(!certificate_.valid()||scene.getWorld()->size()!=0||low<0||high>255||low>high||high-low>2)return finish(false,"unsupported_scene_or_aperture");
  std::vector<const moveit::core::AttachedBody*> attached;actual.getAttachedBodies(attached);
  if(!attached.empty())return finish(false,"attached_geometry");
  for(const auto&v:scene.getCollisionEnvUnpadded()->getLinkPadding())if(v.second!=0)return finish(false,"external_padding");
  for(const auto&v:scene.getCollisionEnvUnpadded()->getLinkScale())if(v.second!=1)return finish(false,"external_scale");
  Eigen::VectorXd q;actual.copyJointGroupPositions(arm_,q);
  if(target.size()!=6||!target.allFinite())return finish(false,"invalid_target");
  const Eigen::VectorXd delta=target-q;int index=-1;
  for(int k=0;k<6;++k)if(std::abs(delta[k])>1e-12){if(index!=-1)return finish(false,"single_joint_only");index=k;}
  if(index<0||std::abs(delta[index])>.5*M_PI/180*.05+1e-12)return finish(false,"recovery_step_limit");
  const auto*joint=model_->getJointOfVariable(arm_->getVariableNames()[index]);
  if(!joint||joint->getType()!=moveit::core::JointModel::REVOLUTE)return finish(false,"revolute_only");
  const double margin=5*M_PI/180;
  for(int k=0;k<6;++k){const auto& bound=model_->getVariableBounds(arm_->getVariableNames()[k]);
    if(target[k]<bound.min_position_+margin||target[k]>bound.max_position_-margin)return finish(false,"joint_limit");}
  const auto* rev=static_cast<const moveit::core::RevoluteJointModel*>(joint);
  const auto joint_tf=actual.getGlobalLinkTransform(joint->getChildLinkModel());
  const Eigen::Vector3d axis=joint_tf.rotation()*rev->getAxis(),pivot=joint_tf.translation();
  const double angle=std::abs(delta[index]),sign=delta[index]>0?1.:-1.;
  result.joint=index;result.low=low;result.high=high;result.source_clearance_m=INFINITY;result.outward_derivative_m_rad=INFINITY;
  std::vector<double> excursion(certificate_.variables.size(),0.);
  auto var=std::find(certificate_.variables.begin(),certificate_.variables.end(),arm_->getVariableNames()[index]);
  if(var==certificate_.variables.end())return finish(false,"unknown_joint");
  excursion[var-certificate_.variables.begin()]=angle;
  bool any_overlap=false;
  for(int raw=low;raw<=high;++raw){
    if(expired())return finish(false,"budget");
    moveit::core::RobotState state(actual);aperture(state,raw);
    auto full_excursion=excursion;std::map<std::string,double> uncertainty;
    for(int other=low;other<=high;++other){moveit::core::RobotState alternate(state);aperture(alternate,other);
      for(size_t i=0;i<certificate_.variables.size();++i){const auto& name=certificate_.variables[i];
        if(name.rfind("op_",0)==0){const double e=std::abs(alternate.getVariablePosition(name)-state.getVariablePosition(name));
          full_excursion[i]=std::max(full_excursion[i],e);uncertainty[name]=std::max(uncertainty[name],e);}}}
    // Covers the continuous independent joint interval between aperture
    // quantization values, not just the integer lookup poses.
    auto aperture_error=[&](const moveit::core::LinkModel* link,const Points& local){
      double radius=0.,error=0.;for(const auto&p:local)radius=std::max(radius,p.norm());
      for(auto*child=link;child;child=child->getParentLinkModel()){
        const auto*j=child->getParentJointModel();
        for(const auto&name:j->getVariableNames())error+=radius*uncertainty[name];
        radius+=child->getJointOriginTransform().translation().norm();
      }return error;
    };
    auto acm=scene.getAllowedCollisionMatrix();acm.setEntry("op_unknown_sweep",true);
    collision_detection::CollisionRequest req;req.group_name="me6_group";req.contacts=true;req.max_contacts=512;req.max_contacts_per_pair=64;
    collision_detection::CollisionResult hits;scene.checkSelfCollision(req,hits,state,acm);
    if(hits.contact_count>=512)return finish(false,"contact_capacity");
    if(hits.collision&&hits.contacts.empty())return finish(false,"missing_contacts");
    // Continue the same outward proof as a reserve contact becomes a tiny
    // positive clearance. An unsigned Lipschitz bound alone is inconclusive
    // at that transition even though the fixed separating plane moves away.
    const auto nearby=certificate_.distances(scene,state,acm,full_excursion,.002);
    for(const auto&pair:nearby.distances)for(const auto&d:pair.second)
      if(std::isfinite(d.distance)&&d.distance<=.002)hits.contacts.try_emplace(pair.first);
    for(const auto&hit:hits.contacts){
      any_overlap=true;
      auto a=hit.first.first,b=hit.first.second;
      const auto*la=model_->getLinkModel(a);const auto*lb=model_->getLinkModel(b);
      if(descendant(la,joint)&&!descendant(lb,joint)){std::swap(a,b);std::swap(la,lb);}
      if(descendant(la,joint)||!descendant(lb,joint)||!sources_.count(b)||sources_.at(b).empty())return finish(false,"not_finger_reserve");
      const auto source=transform(sources_.at(b),state.getGlobalLinkTransform(lb));
      const double source_error=aperture_error(lb,sources_.at(b));
      const auto&pa=parts_.at(a);const auto&pb=parts_.at(b);
      for(size_t i=0;i<pa.size();++i)for(size_t j=0;j<pb.size();++j){
        if(expired())return finish(false,"budget");
        if(pa[i].vertices.empty()||pb[j].vertices.empty()||!pb[j].box)return finish(false,"unsupported_shape");
        const auto ta=state.getCollisionBodyTransform(la,i),tb=state.getCollisionBodyTransform(lb,j);
        const auto av=transform(pa[i].vertices,ta),bv=transform(pb[j].vertices,tb);
        const double box_error=aperture_error(lb,transform(pb[j].vertices,lb->getCollisionOriginTransforms()[j]));
        const double static_error=aperture_error(la,transform(pa[i].vertices,la->getCollisionOriginTransforms()[i]));
        std::vector<Eigen::Vector3d> axes;
        for(int k=0;k<3;++k){axes.push_back(ta.rotation().col(k));axes.push_back(tb.rotation().col(k));}
        for(const auto&c:hit.second)if(c.normal.allFinite()&&c.normal.norm()>.99)axes.push_back(c.normal.normalized());
        for(const auto&n:pa[i].normals)axes.push_back(ta.rotation()*n);
        double source_travel=0.;for(const auto&p:source)source_travel=std::max(source_travel,axis.cross(p-pivot).norm()*angle);
        bool proven=false;
        for(auto n:axes){if(proven)break;for(double sense:{-1.,1.}){
          const Eigen::Vector3d normal=n*sense;
          const double before=gap(av,bv,normal);
          double travel=0.,derivative=INFINITY;
          for(const auto&p:bv){
            const double radius=axis.cross(p-pivot).norm();travel=std::max(travel,radius*angle);
            // Taylor remainder bounds the derivative for EVERY point along
            // the interval, including support-feature changes of the box.
            derivative=std::min(derivative,sign*normal.dot(axis.cross(p-pivot))-radius*angle);
          }
          if(before>travel+box_error*(1+angle)+static_error+1e-6){proven=true;break;}
          const double source_gap=gap(av,source,normal);
          // Entering the normal reserve is still forbidden. Recovery only
          // restores a <=4 mm reserve intrusion while the full original CAD
          // retains >=2 mm separation. No link or current contact is ignored.
          const double source_lower=source_gap-source_travel-source_error*(1+angle)-static_error;
          if(before-box_error-static_error>=-.004&&source_lower>=.002&&derivative-box_error>1e-5){
            proven=true;result.source_clearance_m=std::min(result.source_clearance_m,source_lower);
            result.outward_derivative_m_rad=std::min(result.outward_derivative_m_rad,derivative-box_error);break;
          }
        }}
        if(!proven)return finish(false,"source_clearance_or_outward_proof");
      }
      // Only now is this entire compound pair accounted for separately.
      acm.setEntry(a,b,true);++result.pairs;
    }
    // All remaining original model pairs retain the existing continuous
    // material-point distance certificate. No new intersection is permitted.
    const auto distances=certificate_.distances(scene,state,acm,full_excursion);
    if(expired())return finish(false,"budget");
    if(!certificate_.certifies(distances,acm,full_excursion,state))return finish(false,"other_pair_or_uncertified_interval");
  }
  if(!any_overlap || !std::isfinite(result.source_clearance_m) ||
      !std::isfinite(result.outward_derivative_m_rad))return finish(false,"ordinary_path");
  return finish(!expired(),expired()?"budget":"finger_reserve_retreat");
}
