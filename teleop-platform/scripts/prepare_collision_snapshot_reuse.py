"""Reuse only exact in-range anchor distances measured by the same request.

Run after prepare_collision_snapshot and the request-state adapter patch.
No path result or historical pose is imported, and thresholds are unchanged.
"""
from pathlib import Path
import difflib
import hashlib
import json


def prepare(source: Path):
    edits = []
    path = source / 'collision_retreat.hpp'
    old = path.read_text()
    assert hashlib.sha256(path.read_bytes()).hexdigest() == '59a64e3d45f858bd6b655a0da53bac2b4a0d9b3114dad7313347725026c93667'
    marker = '  const SweepDistanceCertificate::DistanceCache* anchorSeparations('
    assert old.count(marker) == 1
    new = old.replace(marker, '''  bool reuseCheckedAnchor(const planning_scene::PlanningScene& scene,
      const moveit::core::RobotState& actual,const moveit_servo::CollisionSample& sample) {
    // Must follow reset() in this callback. No result can cross request state,
    // model, environment, aperture, matrix, attached-body or world changes.
    if(actual_collision_checked_||!cache_positions_.empty()||!actual_distances_.empty())return false;
    if(!sample.checked||sample.self_collision||sample.scene_collision||
       sample.model.get()!=model_.get()||actual.getRobotModel().get()!=model_.get()||
       sample.environment.get()!=scene.getCollisionEnvUnpadded().get())return false;
    const std::vector<double> positions(actual.getVariablePositions(),actual.getVariablePositions()+model_->getVariableCount());
    if(positions!=sample.positions)return false;
    std::ostringstream matrix;scene.getAllowedCollisionMatrix().print(matrix);
    if(matrix.str()!=sample.matrix)return false;
    std::vector<const moveit::core::AttachedBody*> attached;actual.getAttachedBodies(attached);
    if(!attached.empty()||scene.getWorld()->size()!=0)return false;
    const auto& environment=scene.getCollisionEnvUnpadded();
    for(const auto& entry:environment->getLinkScale())if(entry.second!=1.)return false;
    for(const auto& entry:environment->getLinkPadding())if(entry.second!=0.)return false;
    for(const auto& entry:sample.exact_self_distances)
      if(!std::isfinite(entry.second)||entry.second<=0.)return false;
    actual_distances_=sample.exact_self_distances;
    cache_positions_=positions;cache_matrix_=matrix.str();actual_collision_checked_=true;
    return true;
  }
'''+marker)
    new = new.replace('#include "arm_proximity.hpp"', '#include <moveit_servo/arm_proximity.hpp>\n#include <moveit_servo/collision_monitor.hpp>')
    edits.append((path,old,new))
    path = source / 'servo_cpp_adapter.cpp';old=path.read_text()
    marker='          if(retreat.assess(*scene.operator->(),*actual,smoothed,retreat_deadline,Clock::time_point::min(),hold_excursion)) {'
    assert old.count(marker)==1
    new=old.replace(marker,'          retreat.reuseCheckedAnchor(*scene.operator->(),*actual,servo->getCollisionSample());\n'+marker)
    edits.append((path,old,new))
    patch=[]
    for path,old,new in edits:
        path.write_text(new,newline='\n')
        patch.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='before/'+path.name,tofile='after/'+path.name))
    (source/'collision-snapshot-reuse.patch').write_text(''.join(patch),newline='\n')
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p,_,_ in edits}
