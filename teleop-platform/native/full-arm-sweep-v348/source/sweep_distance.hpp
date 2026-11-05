// SPDX-License-Identifier: BSD-3-Clause
// FCL separation certificates. An uncertified interval uses the original sweep.
#pragma once
#include <moveit/planning_scene/planning_scene.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit/robot_model/aabb.hpp>
#include <moveit/robot_model/revolute_joint_model.hpp>
#include <moveit/collision_detection_fcl/collision_common.hpp>
#include <fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node.h>
#include <map>
#include <mutex>
#include <set>
#include <tuple>
#include <atomic>
#include <functional>

class SweepDistanceCertificate {
  using Joint=moveit::core::JointModel;
  struct Pair { std::string a,b; std::vector<double> reach; size_t ai,bi; };
  moveit::core::RobotModelConstPtr model_;
  std::string group_name_;
  std::vector<Pair> pairs_;
  std::vector<std::vector<double>> link_reaches_;
  std::vector<std::vector<collision_detection::FCLGeometryConstPtr>> geometries_;
  std::vector<std::vector<std::shared_ptr<const fcl::CollisionObjectd>>> objects_;
  bool surface_distances_=false;
  bool warm_surface_queries_=false;
  bool prune_compound_shapes_=true;
  struct SurfaceSeed { Eigen::Vector3d a,b; intptr_t triangle_a,triangle_b; };
  mutable std::map<std::tuple<size_t,size_t,size_t,size_t>,SurfaceSeed> surface_seeds_;
  mutable std::mutex surface_seed_mutex_;
  bool valid_=true;
  // A greedy native RSS descent supplies actual triangle witnesses only.
  // They are search UPPER bounds; the full exact FCL traversal still decides
  // every pair that cannot be excluded by its enclosing lower bound.
  static std::pair<int,int> greedyTriangles(const fcl::BVHModel<fcl::OBBRSSd>& a,
      const fcl::BVHModel<fcl::OBBRSSd>& b,const fcl::Transform3d& tf){
    int ai=0,bi=0;
    auto lower=[&](int i,int j){
      const double d=fcl::distance(tf.linear(),tf.translation(),a.getBV(i).bv,b.getBV(j).bv);
      return std::isfinite(d)?d:0.;
    };
    while(!a.getBV(ai).isLeaf()||!b.getBV(bi).isLeaf()){
      const auto& av=a.getBV(ai);const auto& bv=b.getBV(bi);
      if(bv.isLeaf()||(!av.isLeaf()&&av.bv.size()>=bv.bv.size())){
        const int l=av.leftChild(),r=av.rightChild();ai=lower(l,bi)<=lower(r,bi)?l:r;
      }else{
        const int l=bv.leftChild(),r=bv.rightChild();bi=lower(ai,l)<=lower(ai,r)?l:r;
      }
    }
    return {a.getBV(ai).primitiveId(),b.getBV(bi).primitiveId()};
  }
  std::vector<std::vector<double>> localReaches(const moveit::core::RobotState& anchor,
      const std::vector<double>& excursion)const {
    auto result=link_reaches_;const auto& links=model_->getLinkModelsWithCollisionGeometry();
    for(size_t k=0;k<links.size();++k) {
      const auto* link=links[k];const auto& global=link_reaches_[k];
      const Eigen::Vector3d center=anchor.getGlobalLinkTransform(link)*link->getCenteredBoundingBoxOffset();
      const double sphere=.5*link->getShapeExtentsAtOrigin().norm();double descendants=0.;
      for(auto* child=link;child;child=child->getParentLinkModel()) {
        const auto* joint=child->getParentJointModel();if(joint->getType()==Joint::FIXED)continue;
        const size_t v=std::find(variables.begin(),variables.end(),joint->getVariableNames()[0])-variables.begin();
        const auto& transform=anchor.getGlobalLinkTransform(child);
        const Eigen::Vector3d axis=transform.linear()*static_cast<const moveit::core::RevoluteJointModel*>(joint)->getAxis();
        const double local=(center-transform.translation()).cross(axis).norm()+sphere+descendants;
        result[k][v]=std::min(global[v],local);
        descendants+=global[v]*excursion[v];
      }
    }
    return result;
  }
  double relativeBound(const Pair& p,const std::vector<std::vector<double>>& local,
      const std::vector<double>& excursion)const {
    double bound=1e-6;
    for(size_t i=0;i<excursion.size();++i)
      bound+=std::min(p.reach[i],local[p.ai][i]+local[p.bi][i])*excursion[i];
    return bound;
  }
  // Bound the material-point path length without discarding joint-velocity
  // cancellation. q(s)=q0+s*dq. The midpoint point velocity is affine in the
  // link-local point, so the maximum norm over its collision AABB is attained
  // at a corner. Each serial-chain translation term is a product of rotations:
  // ||p''|| <= sum(length * sum(abs(dq_ancestors))^2)
  //            <= sum(abs(dq_arm)) * global_triangle_travel.
  // Thus ||p'(s)|| <= ||p'(1/2)|| + .5*acceleration_bound for the WHOLE
  // interval. Aperture uncertainty changes point velocity by at most the
  // upstream angular-rate sum times its gripper-induced point excursion.
  std::vector<double> velocityTravels(const moveit::core::JointModelGroup* arm,
      const moveit::core::RobotState& anchor,const Eigen::VectorXd& initial,
      const Eigen::VectorXd& target,const std::vector<double>& excursion)const {
    moveit::core::RobotState mid(anchor);
    mid.setJointGroupPositions(arm,.5*(initial+target));mid.update();
    const auto& names=arm->getVariableNames();std::vector<double> bounds;
    const auto& links=model_->getLinkModelsWithCollisionGeometry();
    for(size_t k=0;k<links.size();++k) {
      const auto* link=links[k];const auto& row=link_reaches_[k];
      double angles=0.,triangle=0.,aperture=0.;bool entered_arm=false;
      std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> axes;
      for(auto* child=link;child;child=child->getParentLinkModel()) {
        const auto* j=child->getParentJointModel();if(j->getType()==Joint::FIXED)continue;
        const auto vi=std::find(variables.begin(),variables.end(),j->getVariableNames()[0]);
        const size_t v=vi-variables.begin();const auto ai=std::find(names.begin(),names.end(),*vi);
        if(ai==names.end()){
          // This tighter proof applies to an arm followed by a tool chain.
          // Unsupported interleaved groups retain the existing bound.
          if(entered_arm)return {};
          aperture+=row[v]*excursion[v];continue;
        }
        entered_arm=true;
        const double d=target[ai-names.begin()]-initial[ai-names.begin()];
        const auto& tf=mid.getGlobalLinkTransform(child);
        axes.emplace_back(d*(tf.linear()*static_cast<const moveit::core::RevoluteJointModel*>(j)->getAxis()),tf.translation());
        angles+=std::abs(d);triangle+=row[v]*std::abs(d);
      }
      double speed=0.;const auto half=.5*link->getShapeExtentsAtOrigin();
      for(int c=0;c<8;++c) {
        Eigen::Vector3d local=link->getCenteredBoundingBoxOffset();
        for(int i=0;i<3;++i)local[i]+=(c&(1<<i)?1.:-1.)*half[i];
        const Eigen::Vector3d p=mid.getGlobalLinkTransform(link)*local;
        Eigen::Vector3d v=Eigen::Vector3d::Zero();
        for(const auto& [omega,origin]:axes)v+=omega.cross(p-origin);
        speed=std::max(speed,v.norm());
      }
      const double bound=speed+angles*(aperture+.5*triangle)+1e-9;
      if(!std::isfinite(bound))return {};
      bounds.push_back(bound);
    }
    return bounds;
  }
  double velocityTravel(const moveit::core::JointModelGroup* arm,
      const moveit::core::RobotState& anchor,const Eigen::VectorXd& initial,
      const Eigen::VectorXd& target,const std::vector<double>& excursion)const {
    const auto bounds=velocityTravels(arm,anchor,initial,target,excursion);
    return bounds.empty()?std::numeric_limits<double>::infinity():*std::max_element(bounds.begin(),bounds.end());
  }
public:
  using DistanceCache=std::map<std::pair<std::string,std::string>,double>;
  // Only a producer that checked the identical unpadded anchor and ACM may
  // supply these exact surface distances. They are used as separation LOWER
  // bounds here, never as a path permission or as an approximate FCL answer.
  using AnchorDistanceSource=std::function<const DistanceCache*(const planning_scene::PlanningScene&,
      const moveit::core::RobotState&,const collision_detection::AllowedCollisionMatrix&)>;
  struct NearPairReject {
    std::set<std::pair<std::string,std::string>> pairs;
    double threshold=0.;bool rejected=false;
  };
  NearPairReject uncertainPairReject(const std::vector<double>& uncertainty,double threshold)const {
    NearPairReject out;out.threshold=threshold;
    if(uncertainty.size()!=variables.size())return out;
    for(const auto& p:pairs_)for(size_t i=0;i<uncertainty.size();++i)
      if(uncertainty[i]>0.&&p.reach[i]>0.){out.pairs.insert(std::minmax(p.a,p.b));break;}
    return out;
  }
  using Parallel=std::function<void(size_t,std::function<void(size_t)>)>;
  std::vector<std::string> variables;
  // FCL's own root RSS encloses every triangle in the immutable native BVH.
  // Its distance is a separation LOWER bound, not an alternate collider.
  // Taking the minimum across all shape pairs covers compound links.
  double surfaceRootLowerBound(const moveit::core::RobotState& state,size_t ai,size_t bi)const {
    if(!surface_distances_)return 0.;
    const auto& links=model_->getLinkModelsWithCollisionGeometry();
    double lower=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<geometries_[ai].size();++i)for(size_t j=0;j<geometries_[bi].size();++j) {
      const auto* a=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[ai][i]->collision_geometry_.get());
      const auto* b=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[bi][j]->collision_geometry_.get());
      if(!a||!b||a->getNumBVs()==0||b->getNumBVs()==0)return 0.;
      // FCL RSS-inl transforms the second BV into the first BV's frame.
      const auto tf=state.getCollisionBodyTransform(links[ai],i).inverse()*state.getCollisionBodyTransform(links[bi],j);
      const double d=fcl::distance(tf.linear(),tf.translation(),a->getBV(0).bv,b->getBV(0).bv);
      if(!std::isfinite(d)||d<0.)return 0.;
      lower=std::min(lower,std::max(0.,d-1e-9));
    }
    return std::isfinite(lower)?lower:0.;
  }
  explicit SweepDistanceCertificate(const moveit::core::RobotModelConstPtr& model,
      const std::string& group_name="e6_gripper_geometry",bool surface_distances=false,bool warm_surface_queries=false,
      bool prune_compound_shapes=true):
      model_(model),group_name_(group_name),surface_distances_(surface_distances),warm_surface_queries_(warm_surface_queries),
      prune_compound_shapes_(prune_compound_shapes) {
    std::map<const Joint*,size_t> indices;
    for(const auto* joint:model->getJointModels()) {
      if(joint->getType()==Joint::FIXED)continue;
      // No extrapolated bounds for unsupported joints or mimic mechanisms.
      if(joint->getType()!=Joint::REVOLUTE || joint->getVariableCount()!=1 || joint->getMimic()) {valid_=false;return;}
      indices[joint]=variables.size();variables.push_back(joint->getVariableNames()[0]);
    }
    const auto& links=model->getLinkModelsWithCollisionGeometry();
    std::vector<std::map<const Joint*,double>> reaches;
    for(const auto* link:links) {
      double radius=0.;
      const auto& shapes=link->getShapes();const auto& origins=link->getCollisionOriginTransforms();
      for(size_t k=0;k<shapes.size();++k) {
        double r=0.;
        if(shapes[k]->type==shapes::BOX) {
          const auto* box=static_cast<const shapes::Box*>(shapes[k].get());
          r=.5*Eigen::Vector3d(box->size[0],box->size[1],box->size[2]).norm();
        } else if(shapes[k]->type==shapes::MESH) {
          const auto* mesh=static_cast<const shapes::Mesh*>(shapes[k].get());
          for(unsigned v=0;v<mesh->vertex_count;++v)r=std::max(r,Eigen::Vector3d(mesh->vertices+3*v).norm());
        } else {valid_=false;return;}
        radius=std::max(radius,origins[k].translation().norm()+r);
      }
      std::map<const Joint*,double> chain;
      for(auto* child=link;child;child=child->getParentLinkModel()) {
        const auto* joint=child->getParentJointModel();
        if(indices.count(joint))chain[joint]=radius;
        // Triangle inequality, independent of all intervening rotations.
        radius+=child->getJointOriginTransform().translation().norm();
      }
      reaches.push_back(chain);
      std::vector<double> row(variables.size(),0.);
      for(const auto& [joint,r]:chain)row[indices[joint]]=r;
      link_reaches_.push_back(std::move(row));
    }
    const auto* group=model->getJointModelGroup(group_name_);
    if(!group){valid_=false;return;}
    const auto& updated=group->getUpdatedLinkModelsSet();
    for(size_t i=0;i<links.size();++i)for(size_t j=i+1;j<links.size();++j) {
      if(!updated.count(links[i])&&!updated.count(links[j]))continue;
      Pair p{links[i]->getName(),links[j]->getName(),std::vector<double>(variables.size(),0.),i,j};
      // A common ancestor is a shared rigid transform: distance is invariant.
      // Only joints on the two branches below their common ancestor matter.
      for(const auto& [joint,r]:reaches[i])if(!reaches[j].count(joint))p.reach[indices[joint]]+=r;
      for(const auto& [joint,r]:reaches[j])if(!reaches[i].count(joint))p.reach[indices[joint]]+=r;
      pairs_.push_back(std::move(p));
    }
    // Reuse MoveIt's native shape/BVH conversion, including compound links.
    // Cache immutable geometry only, never positions or collision outcomes.
    for(const auto* link:links) {
      std::vector<collision_detection::FCLGeometryConstPtr> row;
      std::vector<std::shared_ptr<const fcl::CollisionObjectd>> objects;
      for(size_t k=0;k<link->getShapes().size();++k) {
        // Own an identical shape/BVH instead of sharing MoveIt's mutable FCL
        // AABB cache with its asynchronous monitor. Geometry is then immutable.
        auto shape=shapes::ShapeConstPtr(link->getShapes()[k]->clone());
        // Exact box surface, not a different envelope. Mesh/box GJK distance
        // can jump between features; retreat comparisons need mesh/mesh
        // triangle distances. The default collision/sweep path is unchanged.
        if(surface_distances_ && shape->type==shapes::BOX)
          shape=shapes::ShapeConstPtr(shapes::createMeshFromShape(shape.get()));
        auto geometry=collision_detection::createCollisionGeometry(shape,link,static_cast<int>(k));
        if(!geometry)throw std::runtime_error("Unable to create native collision geometry");
        auto object=std::make_shared<fcl::CollisionObjectd>(geometry->collision_geometry_);
        object->setUserData(nullptr);objects.push_back(std::move(object));
        row.push_back(std::move(geometry));
      }
      geometries_.push_back(std::move(row));
      objects_.push_back(std::move(objects));
    }
  }
  bool valid()const{return valid_;}
  // Same whole-path material-point bound already used by armTravel, retained
  // per collider. The sum for two links bounds their relative distance change.
  // Only fixed-aperture arm paths use it; there is no aperture extrapolation.
  std::vector<double> armLinkTravels(const moveit::core::JointModelGroup* arm,
      const moveit::core::RobotState& anchor,const Eigen::VectorXd& initial,
      const Eigen::VectorXd& target,const std::vector<double>& excursion)const {
    if(!valid_ || excursion.size()!=variables.size())return {};
    for(double d:excursion)if(!std::isfinite(d)||d<0)return {};
    for(size_t i=0;i<variables.size();++i)
      if(std::find(arm->getVariableNames().begin(),arm->getVariableNames().end(),variables[i])==arm->getVariableNames().end() && excursion[i]!=0.)return {};
    return velocityTravels(arm,anchor,initial,target,excursion);
  }
  struct SurfaceWitness {
    Eigen::Vector3d first_world=Eigen::Vector3d::Zero(),second_world=Eigen::Vector3d::Zero();
    double distance=std::numeric_limits<double>::infinity();
    size_t first_shape=0,second_shape=0;
    intptr_t first_triangle=-1,second_triangle=-1;
  };
  using WitnessCache=std::map<std::pair<std::string,std::string>,SurfaceWitness>;
  collision_detection::DistanceResult measure(const planning_scene::PlanningScene& scene,
      const moveit::core::RobotState& state,const collision_detection::AllowedCollisionMatrix& acm,
      DistanceCache* cache=nullptr,const DistanceCache* previous=nullptr,
      double threshold=0.,bool* decreasing=nullptr,const Parallel& parallel={},NearPairReject* early_reject=nullptr,
      double ceiling=std::numeric_limits<double>::infinity(),const DistanceCache* pair_ceilings=nullptr,
      WitnessCache* witnesses=nullptr)const {
    if(witnesses)witnesses->clear();
    if(std::isnan(ceiling)||ceiling<=0.)throw std::invalid_argument("Invalid distance certificate ceiling");
    collision_detection::DistanceResult result;
    if(decreasing)*decreasing=false;
    if(!surface_distances_) {
      collision_detection::DistanceRequest req;req.type=collision_detection::DistanceRequestType::SINGLE;
      req.group_name=group_name_;req.enableGroup(model_);req.acm=&acm;
      scene.getCollisionEnvUnpadded()->distanceSelf(req,result,state);return result;
    }
    const auto& links=model_->getLinkModelsWithCollisionGeometry();
    std::vector<const Pair*> ordered;ordered.reserve(pairs_.size());
    // Filter the already-certified matrix before ordering. Sorting every
    // excluded robot pair repeatedly creates string keys and allocator
    // contention across independent sample queries, even when only one pair
    // remains. This is exactly the same exclusion tested in the loop below.
    for(const auto& p:pairs_){
      collision_detection::AllowedCollision::Type type;
      if(acm.getAllowedCollision(p.a,p.b,type)&&type==collision_detection::AllowedCollision::ALWAYS)continue;
      ordered.push_back(&p);
    }
    if(previous)std::stable_sort(ordered.begin(),ordered.end(),[&](const Pair* a,const Pair* b){
      auto distance=[&](const Pair* p){auto it=previous->find(std::minmax(p->a,p->b));
        return it==previous->end()?std::numeric_limits<double>::infinity():it->second;};
      return distance(a)<distance(b);
    });
    // Prioritize exact queries that can disprove a boost. This only changes
    // query order. An incomplete result is marked and may NEVER permit motion.
    if(early_reject)std::stable_partition(ordered.begin(),ordered.end(),[&](const Pair* p){
      return early_reject->pairs.count(std::minmax(p->a,p->b));
    });
    for(const auto* ptr:ordered) {
      const auto& p=*ptr;
      collision_detection::AllowedCollision::Type type;
      if(acm.getAllowedCollision(p.a,p.b,type)&&type==collision_detection::AllowedCollision::ALWAYS)continue;
      const auto key=std::make_pair(std::min(p.a,p.b),std::max(p.a,p.b));
      double pair_ceiling=ceiling;
      if(pair_ceilings){
        const auto cap=pair_ceilings->find(key);
        if(cap!=pair_ceilings->end()){
          if(!std::isfinite(cap->second)||cap->second<=0.)throw std::invalid_argument("Invalid pair separation ceiling");
          pair_ceiling=std::min(pair_ceiling,cap->second);
        }
      }
      double minimum=std::numeric_limits<double>::infinity();
      if(cache&&cache->count(key))minimum=std::min(cache->at(key),pair_ceiling);
      else {
       struct ShapePair {size_t i,j;double lower;fcl::Transform3d ta,tb;fcl::DistanceResultd seed;};
       std::vector<ShapePair> shape_pairs;
       for(size_t i=0;i<geometries_[p.ai].size();++i)for(size_t j=0;j<geometries_[p.bi].size();++j){
         double lower=0.;
         const auto* a=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[p.ai][i]->collision_geometry_.get());
         const auto* b=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[p.bi][j]->collision_geometry_.get());
         if(prune_compound_shapes_&&a&&b&&a->getNumBVs()&&b->getNumBVs()){
           const auto tf=state.getCollisionBodyTransform(links[p.ai],i).inverse()*state.getCollisionBodyTransform(links[p.bi],j);
           const double d=fcl::distance(tf.linear(),tf.translation(),a->getBV(0).bv,b->getBV(0).bv);
           if(std::isfinite(d)&&d>=0.)lower=std::max(0.,d-1e-9);
         }
         shape_pairs.push_back({i,j,lower,
           collision_detection::transform2fcl(state.getCollisionBodyTransform(links[p.ai],i)),
           collision_detection::transform2fcl(state.getCollisionBodyTransform(links[p.bi],j)),{}});
       }
       if(prune_compound_shapes_)std::stable_sort(shape_pairs.begin(),shape_pairs.end(),
         [](const ShapePair& a,const ShapePair& b){return a.lower<b.lower;});
       // A finite pair_ceiling requests min(exact distance, pair_ceiling), a proven
       // LOWER separation certificate, not an exact far-pair distance. This
       // is used only by the original near-retreat policy, which already
       // clips every adjacent monotonic comparison at that same threshold.
       std::atomic<double> upper{pair_ceiling};
       for(auto& pair:shape_pairs){
         // Establish a current-pose surface bound in root-distance order.
         // A shape already excluded by that bound needs neither a greedy
         // witness descent nor a full FCL query. Warm triangle indices are
         // re-evaluated at this pose; they never provide a lower bound.
         if(prune_compound_shapes_&&pair.lower>0.&&pair.lower>=upper.load())continue;
         const auto* a=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[p.ai][pair.i]->collision_geometry_.get());
         const auto* b=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[p.bi][pair.j]->collision_geometry_.get());
         if(prune_compound_shapes_&&a&&b&&a->getNumBVs()&&b->getNumBVs()){
           const auto tf=pair.ta.inverse()*pair.tb;fcl::DistanceRequestd request;
           request.enable_nearest_points=warm_surface_queries_;bool seeded=false;
           if(warm_surface_queries_){
             SurfaceSeed seed{};bool found=false;
             {std::lock_guard<std::mutex> lock(surface_seed_mutex_);
               const auto entry=surface_seeds_.find(std::make_tuple(p.ai,pair.i,p.bi,pair.j));
               if(entry!=surface_seeds_.end()){seed=entry->second;found=true;}}
             if(found&&seed.triangle_a>=0&&seed.triangle_a<a->num_tris&&seed.triangle_b>=0&&seed.triangle_b<b->num_tris){
               fcl::detail::distancePreprocessOrientedNode(a,b,a->vertices,b->vertices,a->tri_indices,b->tri_indices,
                   seed.triangle_a,seed.triangle_b,tf,request,pair.seed);
               seeded=std::isfinite(pair.seed.min_distance);
             }
           }
           // A valid warm surface witness already bounds this exact search.
           // Cold pairs still obtain the original greedy triangle witness.
           if(!seeded){
             const auto ids=greedyTriangles(*a,*b,tf);
             fcl::detail::distancePreprocessOrientedNode(a,b,a->vertices,b->vertices,a->tri_indices,b->tri_indices,
                 ids.first,ids.second,tf,request,pair.seed);
           }
           if(pair.seed.min_distance<upper.load())upper.store(pair.seed.min_distance);
         }
       }
       // Optional current-pose witnesses. Each lane owns one slot;
       // the caller reduces only after all exact shape queries have joined.
       std::vector<SurfaceWitness> shape_witnesses;
       auto capture_witness=[&](size_t index,const fcl::DistanceResultd& d,bool model_a_frame) {
         if(!witnesses || d.b1<0 || d.b2<0 || !std::isfinite(d.min_distance) ||
            d.min_distance<=0. || !d.nearest_points[0].allFinite() ||
            !d.nearest_points[1].allFinite())return;
         const auto& pair=shape_pairs[index];
         Eigen::Vector3d a=d.nearest_points[0],b=d.nearest_points[1];
         if(model_a_frame){a=pair.ta*a;b=pair.ta*b;}
         if(std::abs((a-b).norm()-d.min_distance)>1e-6)return;
         auto& w=shape_witnesses[index];
         if(d.min_distance>w.distance)return;
         if(p.a==key.first)w={a,b,d.min_distance,pair.i,pair.j,d.b1,d.b2};
         else w={b,a,d.min_distance,pair.j,pair.i,d.b2,d.b1};
       };
       if(witnesses){
         shape_witnesses.resize(shape_pairs.size());
         // Oriented triangle preprocessing stores both witnesses in A's
         // local frame. These seeds are current surface upper bounds only.
         for(size_t i=0;i<shape_pairs.size();++i)capture_witness(i,shape_pairs[i].seed,true);
       }
       auto query_shape=[&](size_t index) {
        const auto& pair=shape_pairs[index];
        // A compound link's distance is the minimum across every shape pair.
        // Only skip a pair when its enclosing RSS proves it cannot improve
        // the current minimum or the explicitly requested pair_ceiling. A capped
        // result is a lower separation certificate; zero/unknown enclosing
        // bounds always get queried. The collision matrix is unchanged.
        if(prune_compound_shapes_&&pair.lower>0.&&pair.lower>=upper.load(std::memory_order_relaxed))return;
        const size_t i=pair.i,j=pair.j;
        const auto* a=geometries_[p.ai][i]->collision_geometry_.get();
        const auto* b=geometries_[p.bi][j]->collision_geometry_.get();
        const auto& ta=pair.ta;const auto& tb=pair.tb;
        fcl::DistanceResultd distance=pair.seed;fcl::DistanceRequestd request;
        request.enable_nearest_points=warm_surface_queries_;
        const auto seed_key=std::make_tuple(p.ai,i,p.bi,j);
        if(warm_surface_queries_&&!prune_compound_shapes_){
          SurfaceSeed seed{};bool seeded=false;
          {std::lock_guard<std::mutex> lock(surface_seed_mutex_);
            const auto found=surface_seeds_.find(seed_key);
            if(found!=surface_seeds_.end()){seed=found->second;seeded=true;}}
          if(seeded){
            // These are points on the immutable triangles, not cached
            // distances/permissions. Transform both at THIS pose to obtain
            // a valid search UPPER bound. FCL still traverses the exact BVH.
            // Oriented mesh traversal stores both witnesses in model A's
            // frame until its native postprocess applies A's transform.
            const auto* first=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(a);
            const auto* second=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(b);
            if(first&&second&&seed.triangle_a>=0&&seed.triangle_a<first->num_tris&&
               seed.triangle_b>=0&&seed.triangle_b<second->num_tris){
              // Recompute the exact closest points on the two previous
              // triangles. Their separation is still only a search upper
              // bound; the complete native BVH query below remains mandatory.
              fcl::detail::distancePreprocessOrientedNode(first,second,first->vertices,second->vertices,
                  first->tri_indices,second->tri_indices,seed.triangle_a,seed.triangle_b,
                  ta.inverse()*tb,request,distance);
            }
          }
        }
        // The result needed is the minimum over this compound link pair,
        // not a separate exact distance for each shape. A surface witness at
        // THIS pose from another shape bounds that global minimum. An explicit
        // pair_ceiling instead requests a capped separation certificate. Let
        // native FCL prune internal BV nodes using this search cutoff,
        // while still traversing every branch that could improve it. No bound
        // or permission is carried from another pose or another link pair.
        const double incumbent=upper.load(std::memory_order_relaxed);
        if(prune_compound_shapes_ && std::isfinite(incumbent) &&
           incumbent>0. && incumbent<distance.min_distance){
          distance=fcl::DistanceResultd(incumbent);
          // This bound has no witness on the current shape pair. Only a new
          // native triangle result may be stored as this pair's warm seed.
          distance.b1=distance.b2=-1;
          distance.nearest_points[0].setZero();distance.nearest_points[1].setZero();
        }
        // This native overload avoids CollisionObject's constructor, which
        // recomputes/writes geometry->computeLocalAABB() over every vertex.
        fcl::distance(a,ta,b,tb,request,distance);
        // FCL's postprocess has converted native query points to world frame.
        capture_witness(index,distance,false);
        double best=upper.load(std::memory_order_relaxed);
        while(distance.min_distance<best&&!upper.compare_exchange_weak(best,distance.min_distance,std::memory_order_relaxed)){}
        if(warm_surface_queries_){
          std::lock_guard<std::mutex> lock(surface_seed_mutex_);
          if(distance.b1>=0 && distance.b2>=0 &&
             distance.min_distance>0. && std::isfinite(distance.min_distance) &&
             distance.nearest_points[0].allFinite() && distance.nearest_points[1].allFinite())
            surface_seeds_[seed_key]={ta.inverse()*distance.nearest_points[0],
                tb.inverse()*distance.nearest_points[1],distance.b1,distance.b2};
          else surface_seeds_.erase(seed_key);
        }
       };
       // Reuse the caller's bounded geometry workers. All queries join before
       // consuming the minimum; no async permission, pose cache or new thread.
       if(parallel&&shape_pairs.size()>1)parallel(shape_pairs.size(),query_shape);
       else for(size_t i=0;i<shape_pairs.size();++i)query_shape(i);
       minimum=upper.load(std::memory_order_relaxed);
       if(witnesses && minimum>0. && minimum<pair_ceiling){
         const SurfaceWitness* best=nullptr;
         for(const auto& w:shape_witnesses)
           if(std::isfinite(w.distance)&&(!best||w.distance<best->distance))best=&w;
         if(best&&std::abs(best->distance-minimum)<=1e-9)(*witnesses)[key]=*best;
       }
       // A capped far result must never masquerade as an exact same-pose
       // cache entry. Exact results strictly below the pair_ceiling remain valid.
       if(cache && (!std::isfinite(pair_ceiling)||minimum<pair_ceiling))(*cache)[key]=minimum;
      }
      collision_detection::DistanceResultsData d;d.distance=minimum;
      result.distances[std::minmax(p.a,p.b)]={d};result.collision|=minimum<=0;
      if(minimum<result.minimum_distance.distance)result.minimum_distance=d;
      if(early_reject&&std::isfinite(minimum)&&minimum<=early_reject->threshold&&early_reject->pairs.count(key)){
        early_reject->rejected=true;return result;
      }
      // Negative proof only: stop on the first approaching pair. The caller
      // must reject this partial result. Accepted paths still measure ALL
      // remaining pairs with the same geometry and numerical tolerance.
      if(previous&&decreasing){
        auto before=previous->find(key);
        if(before!=previous->end()&&minimum+1e-9<std::min(before->second,threshold)){
          *decreasing=true;return result;
        }
      }
    }
    return result;
  }
  // Use the SAME MoveIt/FCL narrow-phase callback for every remaining pair,
  // without constructing a broad-phase manager for the entire robot at every
  // arm x aperture sample. Certificate pruning already did the broad phase.
  // Unsupported group/attached/padding/scale cases keep the original path.
  struct PreparedPairs {
    const collision_detection::CollisionRequest* request=nullptr;
    const collision_detection::AllowedCollisionMatrix* matrix=nullptr;
    std::vector<size_t> indices;
  };
  PreparedPairs preparePairs(const planning_scene::PlanningScene& scene,
      const collision_detection::CollisionRequest& req,
      const moveit::core::RobotState& state,const collision_detection::AllowedCollisionMatrix& acm)const {
    PreparedPairs prepared;
    if(!valid_||req.group_name!=group_name_||req.distance)return prepared;
    std::vector<const moveit::core::AttachedBody*> attached;state.getAttachedBodies(attached);
    if(!attached.empty())return prepared;
    const auto& env=scene.getCollisionEnvUnpadded();
    for(const auto& item:env->getLinkScale())if(item.second!=1.)return prepared;
    for(const auto& item:env->getLinkPadding())if(item.second!=0.)return prepared;
    prepared.request=&req;prepared.matrix=&acm;
    for(size_t k=0;k<pairs_.size();++k) {
      const auto& p=pairs_[k];
      collision_detection::AllowedCollision::Type type;
      if(acm.getAllowedCollision(p.a,p.b,type)&&type==collision_detection::AllowedCollision::ALWAYS)continue;
      prepared.indices.push_back(k);
    }
    return prepared;
  }
  // The caller keeps the request/matrix and scene RO lock alive for this
  // single sweep. Only geometry is persistent; this pair list is rebuilt for
  // each newly certified interval and is never a permit for another request.
  bool checkPairs(const PreparedPairs& prepared,const moveit::core::RobotState& state,
      collision_detection::CollisionResult& result)const {
    if(!prepared.request||!prepared.matrix)return false;
    collision_detection::CollisionData data(prepared.request,&result,prepared.matrix);data.enableGroup(model_);
    const auto& links=model_->getLinkModelsWithCollisionGeometry();
    for(size_t k:prepared.indices) {
      const auto& p=pairs_[k];
      for(size_t i=0;i<geometries_[p.ai].size();++i)for(size_t j=0;j<geometries_[p.bi].size();++j) {
        auto a=*objects_[p.ai][i],b=*objects_[p.bi][j];
        a.setTransform(collision_detection::transform2fcl(state.getCollisionBodyTransform(links[p.ai],i)));a.computeAABB();
        b.setTransform(collision_detection::transform2fcl(state.getCollisionBodyTransform(links[p.bi],j)));b.computeAABB();
        // Same AABB overlap gate as FCL's broad-phase manager.
        if(a.getAABB().overlap(b.getAABB())&&collision_detection::collisionCallback(&a,&b,&data))return true;
      }
    }
    return true;
  }
  // Upper bound for every collider point's arm-induced travel, including the
  // unknown-opening proxy. Same triangle-inequality proof as the pair bounds.
  // This replaces guessed 1.0/0.9/... m reaches, not the 1 mm sampling spacing
  // or 40 mm maximum swept travel. The old bound remains for unknown models.
  double armTravel(const moveit::core::JointModelGroup* arm,
      const Eigen::VectorXd& initial,const Eigen::VectorXd& target)const {
    if(!valid_)return std::numeric_limits<double>::infinity();
    std::vector<double> delta(variables.size(),0.);const auto& names=arm->getVariableNames();
    for(size_t i=0;i<names.size();++i) {
      auto it=std::find(variables.begin(),variables.end(),names[i]);
      if(it==variables.end())return std::numeric_limits<double>::infinity();
      delta[it-variables.begin()]=std::abs(target[i]-initial[i]);
    }
    double travel=0.;for(const auto& row:link_reaches_) {
      double bound=0.;for(size_t i=0;i<row.size();++i)bound+=row[i]*delta[i];
      if(!std::isfinite(bound))return std::numeric_limits<double>::infinity();
      travel=std::max(travel,bound);
    }
    return travel;
  }
  // A local, configuration-aware upper bound on path length. A point's radius
  // about a revolute axis is unchanged by that axis and all its ancestors.
  // Only descendant rotations can change it; their global reach*excursion
  // bounds below cover the entire arm AND aperture interval. The collision
  // AABB's enclosing sphere covers every material point, not only its centre.
  double armTravel(const moveit::core::JointModelGroup* arm,
      const moveit::core::RobotState& anchor,const Eigen::VectorXd& initial,
      const Eigen::VectorXd& target,const std::vector<double>& excursion)const {
    if(!valid_ || excursion.size()!=variables.size())return std::numeric_limits<double>::infinity();
    for(double d:excursion)if(!std::isfinite(d)||d<0)return std::numeric_limits<double>::infinity();
    const auto& arm_names=arm->getVariableNames();
    const auto& links=model_->getLinkModelsWithCollisionGeometry();double travel=0.;
    for(size_t k=0;k<links.size();++k) {
      const auto* link=links[k];const auto& row=link_reaches_[k];
      const auto center=anchor.getGlobalLinkTransform(link)*link->getCenteredBoundingBoxOffset();
      const double sphere=.5*link->getShapeExtentsAtOrigin().norm();
      double descendants=0.,bound=0.;
      for(auto* child=link;child;child=child->getParentLinkModel()) {
        const auto* joint=child->getParentJointModel();
        if(joint->getType()==Joint::FIXED)continue;
        auto vi=std::find(variables.begin(),variables.end(),joint->getVariableNames()[0]);
        if(vi==variables.end())return std::numeric_limits<double>::infinity();
        const size_t v=vi-variables.begin();
        auto ai=std::find(arm_names.begin(),arm_names.end(),*vi);
        if(ai!=arm_names.end()) {
          const auto& transform=anchor.getGlobalLinkTransform(child);
          const Eigen::Vector3d axis=transform.linear()*static_cast<const moveit::core::RevoluteJointModel*>(joint)->getAxis();
          const double local=(center-transform.translation()).cross(axis).norm()+sphere+descendants;
          bound+=std::abs(target[ai-arm_names.begin()]-initial[ai-arm_names.begin()])*std::min(row[v],local);
        }
        descendants+=row[v]*excursion[v];
      }
      if(!std::isfinite(bound))return std::numeric_limits<double>::infinity();
      travel=std::max(travel,bound);
    }
    return std::min(travel,velocityTravel(arm,anchor,initial,target,excursion));
  }
  // Use MoveIt's collision-geometry AABBs as separation LOWER bounds. Only a
  // pair whose lower bound already exceeds its whole-interval motion bound can
  // skip exact FCL. The result contains conservative bounds, not display data.
  // Remaining pairs use FCL SINGLE over ALL shapes; completeness is checked.
  collision_detection::DistanceResult distances(const planning_scene::PlanningScene& scene,
      const moveit::core::RobotState& anchor,const collision_detection::AllowedCollisionMatrix& acm,
      const std::vector<double>& excursion,double clearance=0.,DistanceCache* cache=nullptr,
      const std::vector<double>* link_travel=nullptr,const Parallel& parallel={},
      const DistanceCache* anchor_separations=nullptr,NearPairReject* early_reject=nullptr)const {
    auto refined=acm;collision_detection::DistanceResult broad;
    DistanceCache search_ceilings;
    std::map<std::string,moveit::core::AABB> boxes;
    for(const auto* link:model_->getLinkModelsWithCollisionGeometry()) {
      auto transform=anchor.getGlobalLinkTransform(link);transform.translate(link->getCenteredBoundingBoxOffset());
      boxes[link->getName()].extendWithTransformedBox(transform,link->getShapeExtentsAtOrigin());
    }
    if(excursion.size()!=variables.size())return broad;
    for(double d:excursion)if(!std::isfinite(d)||d<0)return broad;
    const auto local=localReaches(anchor,excursion);
    for(const auto& p:pairs_) {
      collision_detection::AllowedCollision::Type type;
      if(acm.getAllowedCollision(p.a,p.b,type) && type!=collision_detection::AllowedCollision::NEVER)continue;
      double motion_bound=relativeBound(p,local,excursion);
      if(link_travel && link_travel->size()==link_reaches_.size()){
        const double a=(*link_travel)[p.ai],b=(*link_travel)[p.bi];
        if(std::isfinite(a)&&std::isfinite(b)&&a>=0.&&b>=0.)motion_bound=std::min(motion_bound,a+b+1e-6);
      }
      const double bound=motion_bound+clearance;
      const auto& a=boxes.at(p.a);const auto& b=boxes.at(p.b);
      double lower=a.exteriorDistance(b);
      if(anchor_separations){
        const auto known=anchor_separations->find(std::minmax(p.a,p.b));
        if(known!=anchor_separations->end()&&std::isfinite(known->second)&&known->second>0.)
          lower=std::max(lower,std::max(0.,known->second-1e-9));
      }
      // A world-axis box can overlap solely because the entire tool rotated.
      // Reuse MoveIt's bounding boxes in each link's rigid frame as well.
      // Every box still contains the complete collider; exterior distance is
      // a LOWER bound in any orthonormal frame, so taking their maximum is
      // conservative. No geometry is removed and no historical pose is used.
      const auto* la=model_->getLinkModel(p.a);const auto* lb=model_->getLinkModel(p.b);
      if(!(std::isfinite(lower)&&lower>bound))for(const auto* frame:{la,lb}) {
        const auto inverse=anchor.getGlobalLinkTransform(frame).inverse();
        moveit::core::AABB first,second;
        auto ta=inverse*anchor.getGlobalLinkTransform(la);
        auto tb=inverse*anchor.getGlobalLinkTransform(lb);
        ta.translate(la->getCenteredBoundingBoxOffset());tb.translate(lb->getCenteredBoundingBoxOffset());
        first.extendWithTransformedBox(ta,la->getShapeExtentsAtOrigin());
        second.extendWithTransformedBox(tb,lb->getShapeExtentsAtOrigin());
        const double d=first.exteriorDistance(second);
        if(std::isfinite(d))lower=std::max(lower,d);
        if(lower>bound)break;
      }
      if(!(std::isfinite(lower)&&lower>bound) && surface_distances_)
        lower=std::max(lower,surfaceRootLowerBound(anchor,p.ai,p.bi));
      if(std::isfinite(lower)&&lower>bound) {
        refined.setEntry(p.a,p.b,true);collision_detection::DistanceResultsData d;d.distance=lower;
        broad.distances[std::minmax(p.a,p.b)]={d};
      }else if(std::isfinite(bound)&&bound>=0.){
        // distances() already returns conservative separation bounds. A pair
        // farther than its whole-interval travel plus clearance only needs
        // that proof, not a complete far-distance traversal. One outward ULP
        // preserves refinedMatrix's strict > test; below the cutoff FCL is
        // exact. measure() never stores a truncated value in the exact cache.
        search_ceilings[std::minmax(p.a,p.b)]=std::nextafter(bound,std::numeric_limits<double>::infinity());
      }
    }
    auto result=measure(scene,anchor,refined,cache,nullptr,0.,nullptr,parallel,early_reject,
        std::numeric_limits<double>::infinity(),&search_ceilings);
    result.distances.insert(broad.distances.begin(),broad.distances.end());return result;
  }
  bool certifies(const collision_detection::DistanceResult& distances,
      const collision_detection::AllowedCollisionMatrix& acm,const std::vector<double>& excursion,
      const moveit::core::RobotState& anchor)const {
    if(!valid_ || distances.collision || excursion.size()!=variables.size())return false;
    for(double d:excursion)if(!std::isfinite(d)||d<0)return false;
    const auto local=localReaches(anchor,excursion);
    for(const auto& p:pairs_) {
      collision_detection::AllowedCollision::Type type;
      if(acm.getAllowedCollision(p.a,p.b,type)) {
        if(type==collision_detection::AllowedCollision::ALWAYS)continue;
        if(type!=collision_detection::AllowedCollision::NEVER)return false;
      }
      auto it=distances.distances.find(std::minmax(p.a,p.b));
      if(it==distances.distances.end() || it->second.empty())return false;
      const double bound=relativeBound(p,local,excursion);
      for(const auto& d:it->second)if(!std::isfinite(d.distance)||d.distance<=bound)return false;
    }
    return true;
  }
  bool uncertainPairsSeparated(const collision_detection::AllowedCollisionMatrix& matrix,
                              const std::vector<double>& uncertainty)const {
    if(!valid_||uncertainty.size()!=variables.size())return false;
    for(const auto& p:pairs_){
      collision_detection::AllowedCollision::Type type;
      if(matrix.getAllowedCollision(p.a,p.b,type)&&type==collision_detection::AllowedCollision::ALWAYS)continue;
      for(size_t i=0;i<uncertainty.size();++i)if(uncertainty[i]>0.&&p.reach[i]>0.)return false;
    }
    return true;
  }
  // Standard broad/narrow-phase pruning for the WHOLE interval. A nearby pair
  // can require dense checks without rechecking every other proven-separated
  // pair. Missing/nonfinite/conditional pairs remain in the original matrix.
  collision_detection::AllowedCollisionMatrix refinedMatrix(
      const collision_detection::DistanceResult& distances,
      const collision_detection::AllowedCollisionMatrix& acm,
      const std::vector<double>& excursion,const moveit::core::RobotState& anchor,double clearance=0.,
      const std::vector<double>* link_travel=nullptr)const {
    auto result=acm;
    if(!valid_ || distances.collision || excursion.size()!=variables.size())return result;
    for(double d:excursion)if(!std::isfinite(d)||d<0)return result;
    const auto local=localReaches(anchor,excursion);
    for(const auto& p:pairs_) {
      collision_detection::AllowedCollision::Type type;
      if(acm.getAllowedCollision(p.a,p.b,type) && type!=collision_detection::AllowedCollision::NEVER)continue;
      auto it=distances.distances.find(std::minmax(p.a,p.b));
      if(it==distances.distances.end() || it->second.empty())continue;
      double motion_bound=relativeBound(p,local,excursion);
      if(link_travel && link_travel->size()==link_reaches_.size()){
        const double a=(*link_travel)[p.ai],b=(*link_travel)[p.bi];
        if(std::isfinite(a)&&std::isfinite(b)&&a>=0.&&b>=0.)motion_bound=std::min(motion_bound,a+b+1e-6);
      }
      const double bound=motion_bound+clearance;
      bool separated=true;
      for(const auto& d:it->second)if(!std::isfinite(d.distance)||d.distance<=bound){separated=false;break;}
      if(separated)result.setEntry(p.a,p.b,true);
    }
    return result;
  }
};
