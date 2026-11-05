// SPDX-License-Identifier: BSD-3-Clause
#include "sweep_distance.hpp"
#include <array>

  // Sufficient directional proof for a held gripper, including its entire
  // independent joint-error box. Work in each pair's common-ancestor frame:
  // shared upstream motion changes neither distance nor its derivative.
  // For material points a,b, .5*d(|b-a|^2)/ds = (b-a).(vb-va).
  // Rigid-body twists make this bilinear in a,b (the self quadratic terms
  // vanish). Its extrema over two boxes/triangles occur at vertex pairs.
  // Hessian/reach bounds cover the WHOLE arm segment and aperture error, not
  // just samples. BV nodes farther than proximity over that interval need no
  // directional proof; all remaining leaves must pass. Failure only retains
  // the normal slowdown. This does not replace the collision sweep.
  std::set<std::pair<std::string,std::string>> SweepDistanceCertificate::heldRetreatPairs(
      const moveit::core::JointModelGroup* arm,const moveit::core::RobotState& anchor,
      const Eigen::VectorXd& initial,const Eigen::VectorXd& target,
      const collision_detection::AllowedCollisionMatrix& matrix,
      const std::vector<double>& uncertainty,double threshold,
      std::chrono::steady_clock::time_point deadline,const DistanceCache& anchor_distances,
      const std::pair<std::string,std::string>* selected,int refinement)const {
    std::set<std::pair<std::string,std::string>> proved;
    if(!valid_||!surface_distances_||uncertainty.size()!=variables.size())return proved;
    moveit::core::RobotState mid(anchor);mid.setJointGroupPositions(arm,.5*(initial+target));mid.update();
    const auto& links=model_->getLinkModelsWithCollisionGeometry();const auto& names=arm->getVariableNames();
    struct Motion {Eigen::Vector3d omega=Eigen::Vector3d::Zero(),linear=Eigen::Vector3d::Zero();
      double position_error=0.,velocity_error=0.;bool valid=true;};
    for(const auto& pair:pairs_){
      if(selected&&std::pair<std::string,std::string>(std::minmax(pair.a,pair.b))!=*selected)continue;
      collision_detection::AllowedCollision::Type type;
      if(matrix.getAllowedCollision(pair.a,pair.b,type)&&type==collision_detection::AllowedCollision::ALWAYS)continue;
      bool affected=false;for(size_t i=0;i<variables.size();++i)affected|=uncertainty[i]>0.&&pair.reach[i]>0.;
      if(!affected)continue;
      if(std::chrono::steady_clock::now()>=deadline)break;
      auto* common=links[pair.ai];
      std::set<const moveit::core::LinkModel*> parents;
      for(auto* c=links[pair.bi];c;c=c->getParentLinkModel())parents.insert(c);
      while(common&&!parents.count(common))common=common->getParentLinkModel();
      if(!common)continue;
      const Eigen::Isometry3d frame=mid.getGlobalLinkTransform(common).inverse();
      auto motion=[&](size_t index){
        Motion m;double angles=0.,triangle=0.,aperture=0.;bool entered_arm=false;
        for(auto* c=links[index];c!=common;c=c->getParentLinkModel()){
          const auto* j=c->getParentJointModel();if(j->getType()==Joint::FIXED)continue;
          const size_t v=std::find(variables.begin(),variables.end(),j->getVariableNames()[0])-variables.begin();
          const auto ai=std::find(names.begin(),names.end(),variables[v]);
          if(ai==names.end()){
            if(entered_arm){m.valid=false;return m;}
            aperture+=link_reaches_[index][v]*uncertainty[v];continue;
          }
          entered_arm=true;const double d=target[ai-names.begin()]-initial[ai-names.begin()];
          const Eigen::Isometry3d tf=frame*mid.getGlobalLinkTransform(c);
          const Eigen::Vector3d w=d*tf.linear()*static_cast<const moveit::core::RevoluteJointModel*>(j)->getAxis();
          m.omega+=w;m.linear-=w.cross(tf.translation());
          angles+=std::abs(d);triangle+=link_reaches_[index][v]*std::abs(d);
        }
        // Serial rotation products bound nominal p'' by angles*triangle.
        // All aperture joints are downstream of the arm (checked above).
        // Consequently their position error changes velocity by omega_total
        // cross delta_p, not by the sum of individual joint angular speeds.
        // The midpoint omega is exact; its variation on a half segment is
        // bounded by .5*angles^2 via the serial joint-axis derivatives. This
        // preserves cancellation during nearly pure Cartesian translation.
        // A Taylor bound around the midpoint also preserves translation
        // cancellation in the position excursion. The link AABB contains
        // every collider point; affine rigid velocity reaches its maximum
        // norm on a corner. Retain the older bound whenever it is tighter.
        const auto* link=links[index];
        const Eigen::Isometry3d body=frame*mid.getGlobalLinkTransform(link);
        const Eigen::Vector3d center=link->getCenteredBoundingBoxOffset(),half=.5*link->getShapeExtentsAtOrigin();
        double speed=0.;
        for(int bits=0;bits<8;++bits){
          Eigen::Vector3d point=center;
          for(int k=0;k<3;++k)point[k]+=(bits&(1<<k)?1.:-1.)*half[k];
          speed=std::max(speed,(m.omega.cross(body*point)+m.linear).norm());
        }
        m.position_error=std::min(.5*triangle,.5*speed+.125*angles*triangle+1e-12)+aperture;
        m.velocity_error=.5*angles*triangle+(m.omega.norm()+.5*angles*angles)*aperture;
        return m;
      };
      const Motion ma=motion(pair.ai),mb=motion(pair.bi);if(!ma.valid||!mb.valid)continue;
      if(ma.omega.isZero(0.)&&ma.linear.isZero(0.)&&mb.omega.isZero(0.)&&mb.linear.isZero(0.)&&
          ma.velocity_error==0.&&mb.velocity_error==0.){proved.insert(std::minmax(pair.a,pair.b));continue;}
      const double dr=ma.position_error+mb.position_error,dv=ma.velocity_error+mb.velocity_error;
      // An exact anchor witness remains a material-point pair at every later
      // pose. Its distance plus the full arm/error excursion is an UPPER
      // bound on the minimum. Patches above it cannot become closest.
      double limit=threshold;
      const auto known=anchor_distances.find(std::minmax(pair.a,pair.b));
      if(known!=anchor_distances.end()&&std::isfinite(known->second)&&known->second>0.)
        limit=std::min(limit,known->second+2*dr+1e-9);
      // Previous exact queries also supply immutable local material witnesses.
      // Re-transform them at this midpoint: their distance is only an UPPER
      // bound (never a reused distance/permission), including on nonconvex
      // meshes. This avoids treating remote triangle corners as closest.
      {
        std::lock_guard<std::mutex> lock(surface_seed_mutex_);
        for(size_t sa=0;sa<geometries_[pair.ai].size();++sa)for(size_t sb=0;sb<geometries_[pair.bi].size();++sb){
          const auto seed=surface_seeds_.find(std::make_tuple(pair.ai,sa,pair.bi,sb));
          if(seed==surface_seeds_.end())continue;
          const Eigen::Vector3d a=mid.getCollisionBodyTransform(links[pair.ai],sa)*seed->second.a;
          const Eigen::Vector3d b=mid.getCollisionBodyTransform(links[pair.bi],sb)*seed->second.b;
          const double upper=(b-a).norm();if(std::isfinite(upper))limit=std::min(limit,upper+dr+1e-9);
        }
      }
      // For two convex triangle patches, their closest vector w satisfies
      // w.dot(r-w) >= 0 for every material-point separation r. Among pairs
      // that can actually become closest (|r| <= radius), this gives
      // |r-w| <= sqrt(radius^2-|w|^2). A tangent velocity therefore need not
      // be bounded against distant triangle corners. This is a sufficient
      // lower bound only; inconclusive patches still use the original split.
      auto nearPatchSeparates=[&](const auto& pa,const auto& pb,
          const Eigen::Vector3d& wa,const Eigen::Vector3d& wb,double gap){
        const double radius=limit+dr+1e-9;
        if(!(gap>1e-12)||!std::isfinite(gap)||gap>radius)return false;
        const Eigen::Vector3d w=wb-wa;
        // Use the returned witnesses, and decline inconsistent numerical
        // results rather than treating the scalar distance as a certificate.
        const double length=w.norm();
        if(!std::isfinite(length)||std::abs(length-gap)>1e-9||length<1e-12)return false;
        const Eigen::Vector3d v=mb.omega.cross(wb)+mb.linear-ma.omega.cross(wa)-ma.linear;
        double ra2=0.,rb2=0.;
        for(const auto& x:pa)ra2=std::max(ra2,(x-wa).squaredNorm());
        for(const auto& y:pb)rb2=std::max(rb2,(y-wb).squaredNorm());
        const double ra=std::sqrt(ra2),rb=std::sqrt(rb2);
        const double variation=ma.omega.norm()*ra+mb.omega.norm()*rb;
        // Evaluate the actual support at every vertex pair instead of
        // assuming the closest-point routine is numerically exact. The
        // support is affine in both triangles, so its minimum is at vertices.
        double support=std::numeric_limits<double>::infinity();
        for(const auto& x:pa)for(const auto& y:pb)support=std::min(support,w.dot(y-x));
        support-=1e-15;
        const double alpha=w.dot(v)/w.squaredNorm();
        const double tangent=(v-alpha*w).norm();
        const double reach=std::sqrt(std::max(0.,radius*radius+length*length-2*support));
        const double lower=(alpha>=0.?alpha*support:alpha*length*radius)-reach*tangent-radius*variation;
        const double error=dr*(v.norm()+variation)+(radius+dr)*dv;
        return std::isfinite(lower)&&std::isfinite(error)&&lower>=error+1e-15;
      };
      struct PatchBounds {double derivative,radius,speed;};
      auto patchBounds=[&](const auto& pa,const auto& pb){
        // Each rigid velocity depends on one point only. Evaluate it once,
        // then keep squared maxima until the end of the Cartesian product.
        // This is the same vertex bound with two square roots, not two per
        // vertex pair; geometry, subdivisions and error margins are unchanged.
        std::array<Eigen::Vector3d,8> va,vb;size_t ai=0,bi=0;
        for(const auto& x:pa)va[ai++]=ma.omega.cross(x)+ma.linear;
        for(const auto& y:pb)vb[bi++]=mb.omega.cross(y)+mb.linear;
        double low=std::numeric_limits<double>::infinity(),r2=0.,v2=0.;ai=0;
        for(const auto& x:pa){bi=0;for(const auto& y:pb){
          const Eigen::Vector3d r=y-x,v=vb[bi++]-va[ai];
          low=std::min(low,r.dot(v));r2=std::max(r2,r.squaredNorm());v2=std::max(v2,v.squaredNorm());
        }++ai;}
        return PatchBounds{low,std::sqrt(r2),std::sqrt(v2)};
      };
      bool passed=true;size_t visits=0;
      for(size_t sa=0;passed&&sa<geometries_[pair.ai].size();++sa)for(size_t sb=0;passed&&sb<geometries_[pair.bi].size();++sb){
        const auto* a=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[pair.ai][sa]->collision_geometry_.get());
        const auto* b=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(geometries_[pair.bi][sb]->collision_geometry_.get());
        if(!a||!b||!a->getNumBVs()||!b->getNumBVs()){passed=false;break;}
        const Eigen::Isometry3d ta=frame*mid.getCollisionBodyTransform(links[pair.ai],sa);
        const Eigen::Isometry3d tb=frame*mid.getCollisionBodyTransform(links[pair.bi],sb);
        const Eigen::Isometry3d relative=ta.inverse()*tb;
        // BV corners and triangle patches are tiny, bounded records. Keep
        // them on the stack: repeated heap allocations in eight concurrent
        // proof lanes otherwise dominate this geometric arithmetic on X5.
        struct Points {
          std::array<Eigen::Vector3d,8> values;size_t count=0;
          const Eigen::Vector3d* begin()const{return values.data();}
          const Eigen::Vector3d* end()const{return values.data()+count;}
          const Eigen::Vector3d* data()const{return values.data();}
          const Eigen::Vector3d& operator[](size_t i)const{return values[i];}
        };
        auto points=[](const auto* mesh,int index,const Eigen::Isometry3d& tf){
          Points out;const auto& node=mesh->getBV(index);
          if(node.isLeaf()){
            const auto& tri=mesh->tri_indices[node.primitiveId()];
            for(int k=0;k<3;++k)out.values[out.count++]=tf*mesh->vertices[tri[k]];
          }else{
            const auto& box=node.bv.obb;
            for(int k=0;k<8;++k){Eigen::Vector3d p=box.To;
              for(int axis=0;axis<3;++axis)p+=box.axis.col(axis)*box.extent[axis]*(k&(1<<axis)?1.:-1.);
              out.values[out.count++]=tf*p;
            }
          }return out;
        };
        std::vector<std::pair<int,int>> stack;stack.reserve(64);stack.emplace_back(0,0);
        while(passed&&!stack.empty()){
          if((++visits&31)==0&&std::chrono::steady_clock::now()>=deadline){passed=false;break;}
          const auto [ia,ib]=stack.back();stack.pop_back();const auto& na=a->getBV(ia);const auto& nb=b->getBV(ib);
          const double lower=fcl::distance(relative.linear(),relative.translation(),na.bv,nb.bv);
          if(std::isfinite(lower)&&lower>limit+dr+1e-9)continue;
          const auto pa=points(a,ia,ta),pb=points(b,ib,tb);
          if(na.isLeaf()&&nb.isLeaf()){
            Eigen::Vector3d witness_a,witness_b;
            const double exact=fcl::detail::TriangleDistanced::triDistance(pa.data(),pb.data(),witness_a,witness_b);
            if(std::isfinite(exact)&&exact>limit+dr+1e-9)continue;
            if(nearPatchSeparates(pa,pb,witness_a,witness_b,exact))continue;
          }
          const auto bound=patchBounds(pa,pb);
          const double derivative=bound.derivative,radius=bound.radius,speed=bound.speed;
          const double error=dr*speed+(radius+dr)*dv;
          if(std::isfinite(derivative)&&std::isfinite(error)&&derivative>=error+1e-15)continue;
          if(na.isLeaf()&&nb.isLeaf()){
            struct Patch {std::array<Eigen::Vector3d,3> a,b;int depth;};
            constexpr int maximum_patch_depth=20;
            // Depth-first binary subdivision holds at most depth+1 pending
            // patches. This changes storage only, not the subdivision limit.
            std::array<Patch,maximum_patch_depth+2> patches;
            size_t pending=1;patches[0]=Patch{{pa[0],pa[1],pa[2]},{pb[0],pb[1],pb[2]},0};
            while(passed&&pending){
              if((++visits&31)==0&&std::chrono::steady_clock::now()>=deadline){passed=false;break;}
              const auto patch=patches[--pending];Eigen::Vector3d wa,wb;
              const double gap=fcl::detail::TriangleDistanced::triDistance(patch.a.data(),patch.b.data(),wa,wb);
              if(std::isfinite(gap)&&gap>limit+dr+1e-9)continue;
              if(nearPatchSeparates(patch.a,patch.b,wa,wb,gap))continue;
              const auto bound=patchBounds(patch.a,patch.b);
              const double low=bound.derivative,rmax=bound.radius,vmax=bound.speed;
              const double margin=dr*vmax+(rmax+dr)*dv;
              if(std::isfinite(low)&&std::isfinite(margin)&&low>=margin+1e-15)continue;
              if(patch.depth>=maximum_patch_depth){passed=false;break;}
              int side=0,edge=0;double length=0.;
              for(int s=0;s<2;++s)for(int e=0;e<3;++e){
                const auto& tri=s?patch.b:patch.a;const double d=(tri[e]-tri[(e+1)%3]).squaredNorm();
                if(d>length){length=d;side=s;edge=e;}
              }
              auto first=patch,second=patch;first.depth=second.depth=patch.depth+1;
              auto& one=side?first.b:first.a;auto& two=side?second.b:second.a;
              const Eigen::Vector3d midpoint=.5*(one[edge]+one[(edge+1)%3]);
              one[edge]=midpoint;two[(edge+1)%3]=midpoint;
              patches[pending++]=std::move(first);patches[pending++]=std::move(second);
            }
            continue;
          }
          if(nb.isLeaf()||(!na.isLeaf()&&na.bv.size()>=nb.bv.size())){
            stack.emplace_back(na.leftChild(),ib);stack.emplace_back(na.rightChild(),ib);
          }else{stack.emplace_back(ia,nb.leftChild());stack.emplace_back(ia,nb.rightChild());}
        }
      }
      if(!passed&&refinement<4&&std::chrono::steady_clock::now()<deadline){
        // Split an inconclusive error box, never discard part of it. Smaller
        // boxes tighten the derivative enclosure without reducing the actual
        // feedback uncertainty. Both halves must independently pass.
        size_t split=variables.size();double largest=0.;
        for(size_t i=0;i<variables.size();++i)if(pair.reach[i]*uncertainty[i]>largest){
          largest=pair.reach[i]*uncertainty[i];split=i;
        }
        if(split<variables.size()&&largest>1e-9){
          auto half=uncertainty;half[split]*=.5;passed=true;
          const auto key=std::pair<std::string,std::string>(std::minmax(pair.a,pair.b));
          for(int sign:{-1,1}){
            auto shifted=anchor;shifted.setVariablePosition(variables[split],anchor.getVariablePosition(variables[split])+sign*half[split]);shifted.update();
            if(!heldRetreatPairs(arm,shifted,initial,target,matrix,half,threshold,deadline,{},&key,refinement+1).count(key)){
              passed=false;break;
            }
          }
        }
      }
      if(passed&&std::chrono::steady_clock::now()<deadline)proved.insert(std::minmax(pair.a,pair.b));
    }
    return proved;
  }
