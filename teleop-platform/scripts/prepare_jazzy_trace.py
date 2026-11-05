"""Add trajectory provenance to the pinned ServoNode, retaining its algorithm."""
from pathlib import Path
import difflib
import hashlib
import json
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'upstream/moveit2-jazzy'
PIN='1ade0e9dcf50dbbbc3a984b995c786cf12736235'


def once(s,old,new):
    if s.count(old)!=1:raise ValueError('Changed upstream context: '+old)
    return s.replace(old,new,1)


def main():
    assert subprocess.check_output(['git','-C',str(SOURCE),'rev-parse','HEAD'],text=True).strip()==PIN
    original={name:(SOURCE/'moveit_ros/moveit_servo'/name).read_text() for name in
              ['src/servo_node.cpp','include/moveit_servo/servo_node.hpp']}
    cpp=original['src/servo_node.cpp'];header=original['include/moveit_servo/servo_node.hpp']
    header=once(header,'#include <control_msgs/msg/joint_jog.hpp>', '#include <map>\n#include <sstream>\n#include <control_msgs/msg/joint_jog.hpp>')
    header=once(header,'  std::deque<KinematicState> joint_cmd_rolling_window_;',
        '  std::deque<KinematicState> joint_cmd_rolling_window_;\n'
        '  // Output-only provenance keyed by generated trajectory point time.\n'
        '  std::map<int64_t, std::pair<char, int64_t>> input_sources_;')
    for name in ['JointJog','Twist','Pose']:
        marker='  latest_'+{'JointJog':'joint_jog','Twist':'twist','Pose':'pose'}[name]+'_ = *msg;'
        cpp=once(cpp,marker,'  // The loop already holds this lock while consuming the selected input.\n  std::lock_guard<std::mutex> lock_guard(lock_);\n'+marker)
    cpp=re.sub(r'(?m)^([ \t]*)joint_cmd_rolling_window_\.clear\(\);',
        lambda m:m[0]+'\n'+m[1]+'input_sources_.clear();',cpp)
    cpp=once(cpp,'  while (planning_scene_monitor_->getLastUpdateTime().get_clock_type() != node_->get_clock()->get_clock_type() ||\n         servo_node_start > planning_scene_monitor_->getLastUpdateTime())',
        '  while (rclcpp::ok() && !stop_servo_ &&\n         (planning_scene_monitor_->getLastUpdateTime().get_clock_type() != node_->get_clock()->get_clock_type() ||\n          servo_node_start > planning_scene_monitor_->getLastUpdateTime()))')
    cpp=once(cpp,'  KinematicState current_state = servo_->getCurrentRobotState(true /* block for current robot state */);',
        '  if (!rclcpp::ok() || stop_servo_)\n    return;\n'
        '  KinematicState current_state = servo_->getCurrentRobotState(true /* block for current robot state */);')
    before='''          if (const auto msg = composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_))
          {
            trajectory_publisher_->publish(msg.value());
          }'''
    after='''          const char kind = expected_type == CommandType::JOINT_JOG ? 'j' :
                            expected_type == CommandType::TWIST ? 't' : 'p';
          const auto stamp = expected_type == CommandType::JOINT_JOG ? latest_joint_jog_.header.stamp :
                             expected_type == CommandType::TWIST ? latest_twist_.header.stamp : latest_pose_.header.stamp;
          const int64_t input_ns = rclcpp::Time(stamp).nanoseconds();
          const bool stale = (cur_time - rclcpp::Time(stamp)) >=
                             rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
          input_sources_[next_joint_state_value.time_stamp.nanoseconds()] = {stale ? 'h' : kind, input_ns};
          while (!input_sources_.empty() && input_sources_.begin()->first <
                 joint_cmd_rolling_window_.front().time_stamp.nanoseconds())
            input_sources_.erase(input_sources_.begin());
          if (auto msg = composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_))
          {
            // Include the whole window, including the final point omitted from
            // the message because it contributes to its predecessor's velocity.
            std::ostringstream trace;
            trace << "e6_trace_v1";
            for (const auto& state : joint_cmd_rolling_window_)
            {
              const auto found = input_sources_.find(state.time_stamp.nanoseconds());
              trace << ';' << state.time_stamp.nanoseconds() << ',';
              if (found == input_sources_.end()) trace << "s,0";
              else trace << found->second.first << ',' << found->second.second;
            }
            msg->header.frame_id = trace.str();
            trajectory_publisher_->publish(msg.value());
          }'''
    cpp=once(cpp,before,after)
    out=ROOT/'runtime/jazzy-adapter-v35';out.mkdir(parents=True,exist_ok=True)
    contents={'src/servo_node.cpp':cpp,'include/moveit_servo/servo_node.hpp':header}
    patch=''
    for name,data in contents.items():
        target=out/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_text(data,encoding='utf-8',newline='\n')
        path='moveit_ros/moveit_servo/'+name
        patch+=''.join(difflib.unified_diff(original[name].splitlines(True),data.splitlines(True),fromfile='a/'+path,tofile='b/'+path))
    (ROOT/'patches/moveit/0003-jazzy-trajectory-provenance.patch').write_text(patch,encoding='utf-8',newline='\n')
    manifest=dict(pin=PIN,changes=['Output-only point/input stamps','Use the existing mutex for consistent selected-input reads',
        'Make the observed static-startup wait interruptible'],files={p:hashlib.sha256(s.encode()).hexdigest() for p,s in contents.items()},hardware_qualified=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest))


if __name__=='__main__':main()
