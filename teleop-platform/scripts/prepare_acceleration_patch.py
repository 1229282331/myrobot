"""Package the pinned upstream plugin with two reproduced numeric corrections.

The private namespace lets the original and reviewed plugins coexist. It does
not replace the official package or change Servo's kinematics/control logic.
"""
from pathlib import Path
import difflib
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
UPSTREAM=ROOT/'upstream/moveit2-jazzy'
COMMIT='1ade0e9dcf50dbbbc3a984b995c786cf12736235'


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('Upstream patch context differs: '+old)
    return text.replace(old,new,1)


def main():
    assert subprocess.check_output(['git','-C',str(UPSTREAM),'rev-parse','HEAD'],text=True).strip()==COMMIT
    out=ROOT/'runtime/acceleration-filter-v34';out.mkdir(parents=True,exist_ok=True)
    upstream=UPSTREAM/'moveit_core/online_signal_smoothing'
    cpp=(upstream/'src/acceleration_filter.cpp').read_text()
    header=(upstream/'include/moveit/online_signal_smoothing/acceleration_filter.hpp').read_text()
    patched=replace_once(cpp,'// The threshold below which any velocity or position difference is considered zero (rad and rad/s).\nconstexpr double COMMAND_DIFFERENCE_THRESHOLD = 1E-4;\n','')
    patched=replace_once(patched,'  if (positions_offset_.norm() < COMMAND_DIFFERENCE_THRESHOLD &&\n      velocities_offset_.norm() < COMMAND_DIFFERENCE_THRESHOLD)',
        '  // A nonzero position step must accumulate even at fine teleoperation speed.\n'
        '  if (positions_offset_.isZero(0.0) && velocities_offset_.isZero(0.0))')
    patched=replace_once(patched,'  osqp_settings_.verbose = 0;',
        '  osqp_settings_.verbose = 0;\n'
        '  // SI joint increments and acceleration * dt^2 are much smaller than 1e-3.\n'
        '  osqp_settings_.eps_abs = 1e-8;\n'
        '  osqp_settings_.eps_rel = 1e-8;')
    patched=replace_once(patched,'  if (!updateData(osqp_data_, osqp_workspace_, constraints_sparse_, lower_bound, upper_bound))',
        '  // OSQP is initialized with zero motion rows; later update_A does not\n'
        '  // recompute its setup-time scaling. Normalize each equivalent inequality\n'
        '  // by its acceleration displacement so small SI steps remain well scaled.\n'
        '  for (size_t i = 0; i < num_joints_; ++i)\n'
        '  {\n'
        '    const double scale = std::max(std::abs(min_acceleration_limits_[i]),\n'
        '                                  std::abs(max_acceleration_limits_[i])) * update_period * update_period;\n'
        '    if (!(scale > 0.0))\n'
        '      return false;\n'
        '    constraints_sparse_.coeffRef(i, 0) /= scale;\n'
        '    upper_bound[i] /= scale;\n'
        '    lower_bound[i] /= scale;\n'
        '  }\n'
        '  if (!updateData(osqp_data_, osqp_workspace_, constraints_sparse_, lower_bound, upper_bound))')
    patched=replace_once(patched,'#include <moveit/online_signal_smoothing/acceleration_filter.hpp>', '#include "reviewed_acceleration_filter.hpp"')
    patched=replace_once(patched,'namespace online_signal_smoothing\n{','namespace e6_reviewed_smoothing\n{')
    patched=patched.replace('}  // namespace online_signal_smoothing','}  // namespace e6_reviewed_smoothing')
    patched=replace_once(patched,'PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::AccelerationLimitedPlugin,',
        'PLUGINLIB_EXPORT_CLASS(e6_reviewed_smoothing::AccelerationLimitedPlugin,')
    new_header=replace_once(header,'namespace online_signal_smoothing\n{',
        'namespace e6_reviewed_smoothing\n{\nusing online_signal_smoothing::SmoothingBaseClass;')
    new_header=new_header.replace('}  // namespace online_signal_smoothing','}  // namespace e6_reviewed_smoothing')
    (out/'reviewed_acceleration_filter.cpp').write_text(patched,encoding='utf-8',newline='\n')
    (out/'reviewed_acceleration_filter.hpp').write_text(new_header,encoding='utf-8',newline='\n')
    patch=''.join(difflib.unified_diff(cpp.splitlines(True),patched.splitlines(True),
        fromfile='a/moveit_core/online_signal_smoothing/src/acceleration_filter.cpp',
        tofile='b/moveit_core/online_signal_smoothing/src/acceleration_filter.cpp'))
    patch+=''.join(difflib.unified_diff(header.splitlines(True),new_header.splitlines(True),
        fromfile='a/moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/acceleration_filter.hpp',
        tofile='b/moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/acceleration_filter.hpp'))
    (ROOT/'patches/moveit/0002-acceleration-fine-input.patch').write_text(patch,encoding='utf-8',newline='\n')
    manifest=dict(commit=COMMIT,original_cpp_sha256=hashlib.sha256(cpp.encode()).hexdigest(),
        patched_cpp_sha256=hashlib.sha256(patched.encode()).hexdigest(),
        original_header_sha256=hashlib.sha256(header.encode()).hexdigest(),
        patched_header_sha256=hashlib.sha256(new_header.encode()).hexdigest(),
        changes=['Preserve every nonzero fine position increment','Normalize equivalent acceleration constraints for OSQP and use dimensionless tolerances',
                 'Private namespace and plugin registration, original packages remain intact'],hardware_qualified=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest))


if __name__=='__main__':main()
