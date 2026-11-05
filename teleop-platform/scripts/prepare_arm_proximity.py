"""Reproduce the bounded distance-policy patch on pinned directional MoveIt.

Leaves the upstream checkout and installed ROS libraries untouched. The full
original ACM still checks actual collisions; only distance scaling is filtered.
"""
from pathlib import Path
import argparse,difflib,hashlib,json,shutil
from prepare_directional_servo import prepare as directional

def prepare(source,output):
    info=directional(source,output)
    original=(source/'src/collision_monitor.cpp').read_text(encoding='utf8')
    if hashlib.sha256(original.encode()).hexdigest()!='c849a7873c42b6a7f1dfb72b0fcee448e9c7a36888a8c6b2c0d9069249a178c7':
        raise ValueError('Unexpected pinned MoveIt collision_monitor.cpp')
    changed=original
    def replace(old,new):
        nonlocal changed
        if changed.count(old)!=1:raise ValueError('Unexpected pinned collision monitor anchor')
        changed=changed.replace(old,new)
    replace('#include <moveit_servo/collision_monitor.hpp>', '#include <moveit_servo/collision_monitor.hpp>\n#include <moveit_servo/arm_proximity.hpp>')
    replace('  const double log_val = -log(0.001);', '  const double log_val = -log(0.001);\n  const ArmProximityPolicy proximity_policy(planning_scene_monitor_->getRobotModel(), servo_params_.move_group_name);')
    replace('''      locked_scene->getCollisionEnvUnpadded()->checkSelfCollision(
          self_collision_request_, self_collision_result_, robot_state_, locked_scene->getAllowedCollisionMatrix());''', '''      // Actual collisions retain the original ACM; only proximity distances
      // invariant to this joint group's motion are excluded from slowdown.
      proximity_policy.check(locked_scene->getCollisionEnvUnpadded(),
          self_collision_request_, self_collision_result_, robot_state_, locked_scene->getAllowedCollisionMatrix());''')
    (output/'src/collision_monitor.cpp').write_text(changed,encoding='utf8',newline='\n')
    helper=Path(__file__).with_name('arm_proximity.hpp')
    shutil.copy2(helper,output/'include/moveit_servo/arm_proximity.hpp')
    patch=''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),fromfile='upstream/src/collision_monitor.cpp',tofile='arm-proximity/src/collision_monitor.cpp'))
    (output/'arm-proximity.patch').write_text(patch,encoding='utf8',newline='\n')
    info.update(collision_original_sha256=hashlib.sha256(original.encode()).hexdigest(),collision_patched_sha256=hashlib.sha256(changed.encode()).hexdigest(),helper_sha256=hashlib.sha256(helper.read_bytes()).hexdigest(),scope='Distance-only exclusion of joint-group invariant pairs; original ACM still detects actual collisions; directional singularity patch retained')
    (output/'arm-proximity-source.json').write_text(json.dumps(info,indent=2),encoding='utf8')
    return info

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);a=p.parse_args();print(json.dumps(prepare(a.source,a.output)))
