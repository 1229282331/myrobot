// SPDX-License-Identifier: BSD-3-Clause
// Candidate-local proximity escape. No device IO and no persistent permission.
#pragma once
#include "arm_proximity.hpp"
#include "sweep_distance.hpp"
#include <chrono>
#include <sstream>
#include <condition_variable>
#include <functional>
#include <thread>
#include <atomic>

// Bounded reusable compute workers, not a motion queue. The caller joins the
// entire batch before returning and checks its original deadline afterward.
class RetreatDistanceWorkers {
  std::mutex mutex_;
  std::condition_variable changed_,completed_;
  std::vector<std::thread> workers_;
  std::function<void(size_t)> work_;
  size_t generation_=0,next_=0,count_=0,running_=0;
  bool stopping_=false;
  std::exception_ptr error_;
public:
  RetreatDistanceWorkers(){
    for(size_t i=0;i<7;++i)workers_.emplace_back([this]{
      size_t seen=0;std::unique_lock<std::mutex> lock(mutex_);
      for(;;){
        changed_.wait(lock,[&]{return stopping_||generation_!=seen;});
        if(stopping_)return;seen=generation_;
        while(next_<count_){
          const size_t index=next_++;lock.unlock();
          try{work_(index);}catch(...){lock.lock();if(!error_)error_=std::current_exception();lock.unlock();}
          lock.lock();
        }
        if(--running_==0)completed_.notify_one();
      }
    });
  }
  ~RetreatDistanceWorkers(){
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}changed_.notify_all();
    for(auto& worker:workers_)worker.join();
  }
  void run(size_t count,std::function<void(size_t)> work){
    std::unique_lock<std::mutex> lock(mutex_);
    work_=std::move(work);next_=0;count_=count;running_=workers_.size();error_=nullptr;++generation_;
    changed_.notify_all();
    // The caller is the eighth compute lane on the eight-core X5; no per-frame
    // thread is created. Workers sleep outside this bounded geometry batch.
    while(next_<count_){
      const size_t index=next_++;lock.unlock();
      try{work_(index);}catch(...){lock.lock();if(!error_)error_=std::current_exception();lock.unlock();}
      lock.lock();
    }
    completed_.wait(lock,[&]{return running_==0;});work_=nullptr;
    if(error_)std::rethrow_exception(error_);
  }
};

struct CollisionRetreatResult {
  double baseline_scale=1.,selected_scale=1.,elapsed_us=0.,travel_m=0.;
  double minimum_before_m=-1.,minimum_after_m=-1.;
  double actual_collision_us=0.,initial_distances_us=0.,sample_queries_us=0.,interval_checks_us=0.;
  bool attempted=false,boosted=false,verified=false,budget_exhausted=false;
  int evaluations=0,samples=0,identical_path_reuses=0;
  std::string reason="not_needed";
};

class CollisionRetreatPolicy {
  using Clock=std::chrono::steady_clock;
  using Pair=std::pair<std::string,std::string>;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* arm_;
  ArmProximityPolicy proximity_;
  SweepDistanceCertificate certificate_;
  RetreatDistanceWorkers workers_;
  double threshold_;
  SweepDistanceCertificate::DistanceCache actual_distances_;
  // Exact distances are used as upper witnesses by the retreat proof. Keep
  // conservative separation bounds separate: the gripper sweep only needs
  // lower bounds at this same anchor and must not re-query the far pairs.
  SweepDistanceCertificate::DistanceCache actual_separations_;
  std::vector<double> cache_positions_;
  std::string cache_matrix_;
  std::map<std::string,double> cache_hold_excursion_;
  bool actual_collision_checked_=false;
  Eigen::VectorXd verified_target_;
  std::string verified_reason_="retreat";
public:
  CollisionRetreatResult result;
  static constexpr int compute_budget_ms=37;
  CollisionRetreatPolicy(const moveit::core::RobotModelConstPtr& model,const std::string& group,double threshold):
      model_(model),arm_(model->getJointModelGroup(group)),proximity_(model,group),certificate_(model,group,true,true),threshold_(threshold) {}
  void reset(){result=CollisionRetreatResult{};actual_distances_.clear();actual_separations_.clear();cache_positions_.clear();cache_matrix_.clear();cache_hold_excursion_.clear();actual_collision_checked_=false;verified_target_.resize(0);}
  const SweepDistanceCertificate::DistanceCache* anchorSeparations(const planning_scene::PlanningScene& scene,
      const moveit::core::RobotState& actual,const collision_detection::AllowedCollisionMatrix& acm)const {
    // A certificate for a different feedback pose, aperture, model or matrix
    // must not survive, even within one callback. Unmeasured pairs are absent.
    if(!actual_collision_checked_||actual_separations_.empty()||actual.getRobotModel().get()!=model_.get())return nullptr;
    const std::vector<double> positions(actual.getVariablePositions(),actual.getVariablePositions()+model_->getVariableCount());
    if(positions!=cache_positions_)return nullptr;
    std::ostringstream matrix;acm.print(matrix);if(matrix.str()!=cache_matrix_)return nullptr;
    std::vector<const moveit::core::AttachedBody*> attached;actual.getAttachedBodies(attached);
    if(!attached.empty()||scene.getWorld()->size()!=0)return nullptr;
    const auto& environment=scene.getCollisionEnvUnpadded();
    for(const auto& p:environment->getLinkScale())if(p.second!=1.)return nullptr;
    for(const auto& p:environment->getLinkPadding())if(p.second!=0.)return nullptr;
    return &actual_separations_;
  }
  bool assess(const planning_scene::PlanningScene& scene,const moveit::core::RobotState& actual,
      const Eigen::VectorXd& target,Clock::time_point deadline,Clock::time_point reuse_deadline=Clock::time_point::min(),
      const std::map<std::string,double>& hold_excursion={}) {
    auto started=Clock::now();++result.evaluations;
    auto finish=[&](bool passed,const std::string& reason){
      result.elapsed_us+=std::chrono::duration<double,std::micro>(Clock::now()-started).count();
      result.reason=reason;return passed;
    };
    auto expired=[&](){if(Clock::now()>=deadline){result.budget_exhausted=true;return true;}return false;};
    // A proof completed before its computation budget can still be consumed
    // within this callback's original execution deadline. A different path
    // never receives this allowance and must meet the computation budget.
    if(Clock::now()>=std::max(deadline,reuse_deadline)){result.budget_exhausted=true;return finish(false,"budget");}
    std::vector<const moveit::core::AttachedBody*> attached;actual.getAttachedBodies(attached);
    if(!certificate_.valid()||!attached.empty()||scene.getWorld()->size()!=0)
      return finish(false,"unsupported_scene");
    const auto& environment=scene.getCollisionEnvUnpadded();
    for(const auto& p:environment->getLinkScale())if(p.second!=1.)return finish(false,"scaled_geometry");
    for(const auto& p:environment->getLinkPadding())if(p.second!=0.)return finish(false,"padded_geometry");
    Eigen::VectorXd q;actual.copyJointGroupPositions(arm_,q);
    if(target.size()!=q.size()||!target.allFinite()||(target-q).cwiseAbs().maxCoeff()<1e-12)
      return finish(false,"no_motion");
    std::vector<double> excursion(certificate_.variables.size(),0.);
    const auto& names=arm_->getVariableNames();
    for(size_t i=0;i<names.size();++i){
      auto v=std::find(certificate_.variables.begin(),certificate_.variables.end(),names[i]);
      if(v==certificate_.variables.end())return finish(false,"unsupported_joint");
      excursion[v-certificate_.variables.begin()]=std::abs(target[i]-q[i]);
    }
    const double travel=certificate_.armTravel(arm_,actual,q,target,excursion);result.travel_m=travel;
    if(!std::isfinite(travel)||travel>.04)return finish(false,"travel_limit");
    const auto original=scene.getAllowedCollisionMatrix();
    // Reuse only identical ACTUAL-state queries within this callback. A final
    // smoothed path has its own interval proof and endpoint checks. No cache
    // survives reset(), a changed aperture/state, or a changed collision matrix.
    std::vector<double> positions(actual.getVariablePositions(),actual.getVariablePositions()+model_->getVariableCount());
    std::ostringstream matrix;original.print(matrix);
    if(positions!=cache_positions_||matrix.str()!=cache_matrix_){
      actual_distances_.clear();actual_separations_.clear();actual_collision_checked_=false;verified_target_.resize(0);cache_positions_=positions;cache_matrix_=matrix.str();
    }
    if(cache_hold_excursion_!=hold_excursion){
      verified_target_.resize(0);cache_hold_excursion_=hold_excursion;
    }
    if(verified_target_.size()==target.size() && (verified_target_.array()==target.array()).all()){
      if(Clock::now()>=std::max(deadline,reuse_deadline)){result.budget_exhausted=true;return finish(false,"budget");}
      ++result.identical_path_reuses;return finish(true,verified_reason_);
    }
    if(expired())return finish(false,"budget");
    collision_detection::CollisionRequest cr;cr.group_name=arm_->getName();
    collision_detection::CollisionResult collision;
    if(!actual_collision_checked_){
      const auto stage_started=Clock::now();
      environment->checkSelfCollision(cr,collision,actual,original);
      result.actual_collision_us+=std::chrono::duration<double,std::micro>(Clock::now()-stage_started).count();
      if(collision.collision)return finish(false,"actual_collision");
      actual_collision_checked_=true;
    }
    auto acm=proximity_.distanceMatrix(original);
    const auto link_travel=certificate_.armLinkTravels(arm_,actual,q,target,excursion);
    const auto distances_started=Clock::now();
    auto combined_excursion=excursion,uncertainty=std::vector<double>(excursion.size(),0.);
    for(const auto& entry:hold_excursion){
      auto it=std::find(certificate_.variables.begin(),certificate_.variables.end(),entry.first);
      if(it==certificate_.variables.end()||!std::isfinite(entry.second)||entry.second<0.)return finish(false,"invalid_hold_geometry");
      const size_t index=it-certificate_.variables.begin();
      if(excursion[index]!=0.)return finish(false,"invalid_hold_geometry");
      combined_excursion[index]=uncertainty[index]=entry.second;
    }
    // Do not apply an arm-only bound to possible residual finger motion.
    const auto* bounds=hold_excursion.empty()?&link_travel:nullptr;
    SweepDistanceCertificate::Parallel parallel=[&](size_t n,std::function<void(size_t)> work){workers_.run(n,std::move(work));};
    auto distances=certificate_.distances(scene,actual,acm,combined_excursion,threshold_,&actual_distances_,bounds,parallel);
    if(!distances.collision)for(const auto& item:distances.distances){
      double lower=std::numeric_limits<double>::infinity();
      for(const auto& value:item.second)lower=std::min(lower,value.distance);
      if(std::isfinite(lower)&&lower>0.)actual_separations_[item.first]=std::max(actual_separations_[item.first],lower);
    }
    result.initial_distances_us+=std::chrono::duration<double,std::micro>(Clock::now()-distances_started).count();
    if(expired())return finish(false,"budget");
    if(distances.collision)return finish(false,"actual_collision");
    // Pairs outside this matrix are proved to stay beyond the slowdown
    // threshold over the WHOLE requested interval, not just its endpoints.
    auto narrow=certificate_.refinedMatrix(distances,acm,combined_excursion,actual,threshold_,bounds);
    // Directional proof covers all static aperture errors. Independently
    // require continuous collision separation over the arm AND error box.
    if(!hold_excursion.empty()&&!certificate_.certifies(distances,narrow,combined_excursion,actual))
      return finish(false,"uncertified_hold_interval");
    std::map<Pair,double> previous,initial;
    for(const auto& item:distances.distances){
      collision_detection::AllowedCollision::Type type;
      if(narrow.getAllowedCollision(item.first.first,item.first.second,type)&&type==collision_detection::AllowedCollision::ALWAYS)continue;
      double d=std::numeric_limits<double>::infinity();
      for(const auto& v:item.second)d=std::min(d,v.distance);
      if(!std::isfinite(d)||d<=0)return finish(false,"invalid_distance");
      initial[item.first]=previous[item.first]=d;
    }
    bool near=false;double minimum=std::numeric_limits<double>::infinity();
    for(const auto& p:initial){near|=p.second<threshold_;minimum=std::min(minimum,p.second);}
    result.minimum_before_m=std::isfinite(minimum)?minimum:-1.;
    // The asynchronous monitor can still carry its earlier near-state scale.
    // A completely separated whole interval already has a clearance proof.
    // Otherwise continue the same exact samples/continuous collision proof;
    // outside proximity require strict monotonic escape from every remaining
    // pair, so a stale monitor cannot introduce a speed notch on the way out.
    if(initial.empty()){verified_target_=target;verified_reason_="clearance_restored";return finish(true,verified_reason_);}
    const int steps=std::max(1,int(std::ceil(travel/.001)));
    auto segment_excursion=excursion;for(auto& v:segment_excursion)v/=steps;
    moveit::core::RobotState previous_state(actual),test(actual);
    struct Sample {collision_detection::DistanceResult distances;bool queried=false,approaching=false;};
    std::vector<Sample> samples(steps);
    std::atomic<bool> early_reject{false};
    auto query=[&](size_t index){
      if(early_reject.load(std::memory_order_relaxed)||Clock::now()>=deadline)return;
      moveit::core::RobotState pose(actual);
      pose.setJointGroupPositions(arm_,q+(target-q)*(double(index+1)/steps));pose.update();
      // Independent queries can reject a decrease from the initial pose early.
      // Full adjacent-sample monotonic and interval checks remain ordered below.
      samples[index].distances=certificate_.measure(scene,pose,narrow,nullptr,&initial,threshold_,&samples[index].approaching,
          steps<=2?parallel:SweepDistanceCertificate::Parallel{});
      samples[index].queried=true;
      if(samples[index].approaching||samples[index].distances.collision)early_reject.store(true,std::memory_order_relaxed);
    };
    for(int i=1;i<=steps;++i){
      if(expired())return finish(false,"budget");
      // Continuous non-collision certificate between every pair of samples.
      // Incomplete distances or uncertain intervals retain baseline slowdown.
      const auto certificate_started=Clock::now();
      const bool certified=certificate_.certifies(distances,narrow,segment_excursion,previous_state);
      result.interval_checks_us+=std::chrono::duration<double,std::micro>(Clock::now()-certificate_started).count();
      if(!certified)
        return finish(false,"uncertified_interval");
      test.setJointGroupPositions(arm_,q+(target-q)*(double(i)/steps));test.update();
      if(i==1){
        const auto query_started=Clock::now();
        if(steps<=2){
          // Small near-boundary steps benefit more from parallel compound
          // queries than from two serial-shape sample lanes. A demonstrated
          // approach rejects immediately; valid retreats still query both.
          for(int j=0;j<steps&&!early_reject.load(std::memory_order_relaxed);++j)query(j);
        }else workers_.run(steps,query);
        result.sample_queries_us+=std::chrono::duration<double,std::micro>(Clock::now()-query_started).count();
        for(const auto& sample:samples)if(sample.queried)++result.samples;
      }
      if(expired())return finish(false,"budget");
      if(early_reject.load(std::memory_order_relaxed))return finish(false,"approaching_pair");
      if(!samples[i-1].queried)return finish(false,"missing_sample");
      auto next=std::move(samples[i-1].distances);
      if(next.collision)return finish(false,"candidate_collision");
      if(samples[i-1].approaching)return finish(false,"approaching_pair");
      for(const auto& p:initial){
        auto found=next.distances.find(p.first);
        if(found==next.distances.end()||found->second.empty())return finish(false,"missing_pair");
        double d=std::numeric_limits<double>::infinity();
        for(const auto& v:found->second)d=std::min(d,v.distance);
        if(!std::isfinite(d)||d<=0)return finish(false,"invalid_distance");
        // Check every potentially limiting pair, including switches of the
        // nearest shape. Numerical tolerance is 1 nm, not a motion allowance.
        if(d+1e-9<(near?std::min(previous[p.first],threshold_):previous[p.first]))return finish(false,"approaching_pair");
        previous[p.first]=d;
      }
      distances=std::move(next);previous_state=test;
    }
    bool improving=false;minimum=std::numeric_limits<double>::infinity();
    for(const auto& p:initial){
      improving|=(!near||p.second<threshold_)&&previous[p.first]>p.second+1e-7;
      minimum=std::min(minimum,previous[p.first]);
    }
    result.minimum_after_m=std::isfinite(minimum)?minimum:-1.;
    if(improving&&!certificate_.uncertainPairsSeparated(narrow,uncertainty)){
      // First reject nominal approaches, then spend work on the robust hold
      // proof. Each independent restricting pair uses the bounded workers.
      const auto affected=certificate_.uncertainPairReject(uncertainty,threshold_).pairs;
      std::vector<Pair> pending;
      for(const auto& pair:affected){
        collision_detection::AllowedCollision::Type type;
        if(!(narrow.getAllowedCollision(pair.first,pair.second,type)&&type==collision_detection::AllowedCollision::ALWAYS))pending.push_back(pair);
      }
      // Subdivide the same arm path to tighten its derivative bounds. Every
      // segment retains the FULL aperture error; nothing is sampled away.
      const size_t pieces=std::min(size_t(16),size_t(std::max(1,int(std::ceil(travel/.00015)))));
      // For tiny starts, subdividing an already tiny arm interval repeats
      // the same loose aperture bound. Partition that error box instead.
      // All cells and the entire arm interval must pass, on the same eight
      // workers; no uncertainty is dropped and no certificate is reused.
      const size_t cells=std::max(size_t(1),(8+pending.size()*pieces-1)/std::max(size_t(1),pending.size()*pieces));
      struct Task {Pair pair;size_t piece;SweepDistanceCertificate::HoldBox box;};
      std::vector<Task> tasks;
      for(const auto& pair:pending){
        const auto boxes=certificate_.heldErrorBoxes(pair,uncertainty,cells);
        if(boxes.empty())return finish(false,"invalid_hold_partition");
        for(size_t piece=0;piece<pieces;++piece)for(const auto& box:boxes)tasks.push_back({pair,piece,box});
      }
      std::vector<unsigned char> passes(tasks.size(),0);
      const SweepDistanceCertificate::DistanceCache no_anchor_cache;
      // An inconclusive optional speed-up must leave time for the ordinary
      // (slower) candidate's collision check, rather than consume its deadline
      // and turn proximity slowdown into a direction stop.
      const auto proof_deadline=deadline-std::chrono::milliseconds(2);
      workers_.run(passes.size(),[&](size_t i){
        const auto& task=tasks[i];const size_t piece=task.piece;
        const Eigen::VectorXd left=q+(target-q)*(double(piece)/pieces),right=q+(target-q)*(double(piece+1)/pieces);
        moveit::core::RobotState segment(actual);segment.setJointGroupPositions(arm_,left);
        for(size_t v=0;v<certificate_.variables.size();++v)if(task.box.offset[v]!=0.)
          segment.setVariablePosition(certificate_.variables[v],actual.getVariablePosition(certificate_.variables[v])+task.box.offset[v]);
        segment.update();
        passes[i]=certificate_.heldRetreatPairs(arm_,segment,left,right,narrow,task.box.half,threshold_,proof_deadline,
            no_anchor_cache,&task.pair,task.box.depth).count(task.pair);
      });
      if(expired())return finish(false,"budget");
      if(std::find(passes.begin(),passes.end(),0)!=passes.end())return finish(false,"gripper_proximity_uncertain");
    }
    if(improving){verified_target_=target;verified_reason_=near?"retreat":"clearance_restored";}
    return finish(improving,improving?verified_reason_:"no_improvement");
  }
};
