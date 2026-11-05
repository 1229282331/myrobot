"""Copy the pinned MoveIt package and apply a bounded directional escape patch.

The installed ROS package and upstream checkout remain untouched. The output
contains the original license, exact diff and source hashes for reproduction.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

PIN='1ade0e9dcf50dbbbc3a984b995c786cf12736235'

def prepare(source, output):
    original=(source/'src/utils/common.cpp').read_text(encoding='utf8')
    # Compare canonical upstream bytes, independent of Windows checkout EOL.
    digest=hashlib.sha256(original.encode()).hexdigest()
    expected='35db0297387a7bb2bfac3077733a4d09f68ccd55b644353e8ef03b12028d7f0a'
    if digest!=expected:raise ValueError('Unexpected pinned MoveIt common.cpp')
    changed=original
    def replace(old,new):
        nonlocal changed
        if changed.count(old)!=1:raise ValueError('Upstream patch anchor changed')
        changed=changed.replace(old,new)
    replace('  const Eigen::MatrixXd matrix_s = current_svd.singularValues().asDiagonal();',
'''  // E6 bounded directional escape: never invert a numerically singular Jacobian.
  if (!current_svd.singularValues().allFinite() ||
      current_svd.singularValues().tail(1)(0) <= 1e-10)
    return {0.0, StatusCode::HALT_FOR_SINGULARITY};
  const Eigen::MatrixXd matrix_s = current_svd.singularValues().asDiagonal();''')
    replace('  next_joint_angles += pseudo_inverse * delta_x;',
'''  const Eigen::VectorXd original_joint_angles = next_joint_angles;
  next_joint_angles += pseudo_inverse * delta_x;''')
    replace('  // Compute the scale based on the current condition number.',
'''  // E6: above the ordinary threshold only a finite, improving retreat is
  // allowed, and only inside the existing leaving-threshold band. The
  // original approach threshold is unchanged. Probe the requested direction
  // at the maximum escape scale; the adapter also checks the final smoothed
  // candidate before it can become an executable target.
  const bool escape = current_condition_number >= hard_stop_singularity_threshold;
  if (escape)
  {
    if (moving_towards_singularity || current_condition_number >= upper_threshold)
      return {0.0, StatusCode::HALT_FOR_SINGULARITY};
    const Eigen::VectorXd retreat_joints = original_joint_angles +
        0.25 * pseudo_inverse * target_delta_x;
    if (!retreat_joints.allFinite())
      return {0.0, StatusCode::HALT_FOR_SINGULARITY};
    moveit::core::RobotState probe(*robot_state);
    probe.setJointGroupPositions(joint_model_group, retreat_joints);
    const Eigen::JacobiSVD<Eigen::MatrixXd> retreat_svd(probe.getJacobian(joint_model_group));
    const auto sv = retreat_svd.singularValues();
    if (!sv.allFinite() || sv(dims - 1) <= 1e-10 ||
        sv(0) / sv(dims - 1) >= current_condition_number - 1e-8)
      return {0.0, StatusCode::HALT_FOR_SINGULARITY};
  }

  // Compute the scale based on the current condition number.''')
    replace('  const bool is_below_hard_stop_limit = current_condition_number < hard_stop_singularity_threshold;',
            '  const bool is_below_hard_stop_limit = current_condition_number < upper_threshold;')
    replace('  return std::make_pair(velocity_scale, servo_status);',
'''  if (escape)
    velocity_scale = std::min(0.25, velocity_scale);
  return std::make_pair(velocity_scale, servo_status);''')
    if output.exists():raise ValueError('Use a new build directory; preserve prior evidence')
    shutil.copytree(source,output)
    (output/'src/utils/common.cpp').write_text(changed,encoding='utf8',newline='\n')
    patch=''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),
        fromfile='upstream/src/utils/common.cpp',tofile='directional/src/utils/common.cpp'))
    (output/'directional.patch').write_text(patch,encoding='utf8',newline='\n')
    info=dict(upstream='https://github.com/moveit/moveit2',commit=PIN,
        original_sha256=digest,patched_sha256=hashlib.sha256(changed.encode()).hexdigest(),
        scope='Only singularity direction/escape scaling; official IK/smoothing/collision/joint limits retained')
    (output/'directional-source.json').write_text(json.dumps(info,indent=2),encoding='utf8')
    return info

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();print(json.dumps(prepare(a.source,a.output)))
