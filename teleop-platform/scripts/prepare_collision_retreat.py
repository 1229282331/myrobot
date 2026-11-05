"""Pinned Servo extension: optional candidate-local scale selector, default unchanged."""
from pathlib import Path
import argparse,difflib,hashlib,json
from prepare_arm_proximity import prepare as proximity

def prepare(source,output):
    info=proximity(source,output)
    header=output/'include/moveit_servo/servo.hpp';original=header.read_text();modified=original
    anchor='  KinematicState getNextJointState(const moveit::core::RobotStatePtr& robot_state, const ServoInput& command);'
    assert modified.count(anchor)==1
    modified=modified.replace('#include <variant>','#include <variant>\n#include <functional>')
    modified=modified.replace(anchor,anchor+'''
  // Optional selector for the final acceleration-smoothed candidate. Rejection
  // restores the committed smoother state for the original scaled candidate.
  // Collision halt (zero) is never overridable.
  using CollisionScaleSelector = std::function<double(double, const Eigen::VectorXd&)>;
  void setCollisionScaleSelector(CollisionScaleSelector selector) { collision_scale_selector_ = std::move(selector); }
''')
    modified=modified.replace('private:\n','private:\n  CollisionScaleSelector collision_scale_selector_;\n',1)
    header.write_text(modified,newline='\n');patch=''.join(difflib.unified_diff(original.splitlines(True),modified.splitlines(True),fromfile='proximity/include/moveit_servo/servo.hpp',tofile='retreat/include/moveit_servo/servo.hpp'))
    path=output/'src/servo.cpp';original=path.read_text();modified=original
    start=modified.index('KinematicState Servo::getNextJointState(');end=modified.index('\nstd::optional<Eigen::Isometry3d>',start)
    body=modified[start:end]
    body=body.replace('  if (collision_velocity_scale_ >', '  const auto command_status = servo_status_;\n  double collision_scale = collision_velocity_scale_.load();\n  if (collision_velocity_scale_ >',1)
    body=body.replace('if (collision_velocity_scale_ > 0 && collision_velocity_scale_ < 1)','if (collision_scale > 0 && collision_scale < 1)').replace('else if (collision_velocity_scale_ == 0)','else if (collision_scale == 0)')
    anchor='    // Apply collision scaling to the joint position delta'
    assert body.count(anchor)==1
    body=body.replace(anchor,'''    // Trial the final smoothed candidate once, after IK and velocity limits.
    // AccelerationLimitedPlugin.reset restores committed position/velocity;
    // do not assume this transactional behavior for other smoothing plugins.
    if (collision_scale_selector_ && collision_scale > 0.0 && collision_scale < 1.0 &&
        servo_params_.use_smoothing &&
        servo_params_.smoothing_filter_plugin_name == "e6_reviewed_smoothing::AccelerationLimitedPlugin" &&
        (command_status == StatusCode::NO_WARNING ||
         command_status == StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY ||
         command_status == StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY))
    {
      const auto trial_halts = jointVariablesToHalt(target_state.positions, target_state.velocities,
                                                   joint_bounds, joint_limit_margins_);
      if (trial_halts.empty())
      {
        auto trial = target_state;
        doSmoothing(trial);
        const double selected = collision_scale_selector_(collision_scale, trial.positions);
        if (selected == 1.0)
        {
          servo_status_ = command_status;
          return trial;
        }
        resetSmoothing(current_state);
      }
    }
'''+anchor)
    body=body.replace('current_state.positions + collision_velocity_scale_ *','current_state.positions + collision_scale *')
    modified=modified[:start]+body+modified[end:];path.write_text(modified,newline='\n')
    patch+=''.join(difflib.unified_diff(original.splitlines(True),modified.splitlines(True),fromfile='proximity/src/servo.cpp',tofile='retreat/src/servo.cpp'))
    (output/'collision-retreat.patch').write_text(patch,newline='\n')
    info['retreat_servo_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    info['retreat_header_sha256']=hashlib.sha256(header.read_bytes()).hexdigest()
    info['retreat_scope']='One final acceleration-smoothed candidate proof; rejected trials restore committed smoother state and use original scaled candidate; collision halt/IK/joint limits retained'
    (output/'collision-retreat-source.json').write_text(json.dumps(info,indent=2));return info

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);a=p.parse_args();print(json.dumps(prepare(a.source,a.output)))
