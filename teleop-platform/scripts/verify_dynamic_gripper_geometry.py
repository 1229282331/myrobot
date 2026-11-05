from pathlib import Path
import argparse,hashlib,sys,json,xml.etree.ElementTree as ET,itertools,numpy as np,trimesh
parser=argparse.ArgumentParser();parser.add_argument('--assets',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();assets=args.assets;out=args.output;out.mkdir(parents=True,exist_ok=True)
g=json.loads((assets/'gripper-kinematics.json').read_text());tree=ET.parse(assets/'robot.urdf');joints={j.find('child').get('link'):j for j in tree.findall('joint')};names=g['joint_names'];dq=np.abs(np.diff(np.array(g['joints_rad_by_raw']),axis=0));checks=[]
for box in g['local_boxes']:
 link=box['link'];mesh=trimesh.load(assets/(link+'.stl'),force='mesh',process=False);pose=np.array(box['box_to_link']);size=np.array(box['size_m']);local=(mesh.vertices-pose[:3,3])@pose[:3,:3]
 clearance=float(np.min(size/2-np.abs(local)));assert clearance>=.0049999
 corners=np.array(list(itertools.product((-1,1),repeat=3)))*size/2;corners=corners@pose[:3,:3].T+pose[:3,3]
 radius=float(np.linalg.norm(corners,axis=1).max());bound=np.zeros(255);child=link;chain=[]
 while child!='omnipicker':
  j=joints[child];name=j.get('name')
  if name in names:bound+=radius*dq[:,names.index(name)];chain.append({'joint':name,'reach_m':radius})
  origin=j.find('origin');radius+=np.linalg.norm(list(map(float,origin.get('xyz','0 0 0').split()))) if origin is not None else 0
  child=j.find('parent').get('link')
 maximum=float(max(bound));assert maximum<.001
 checks.append({'link':link,'source_vertices':len(mesh.vertices),'minimum_local_coverage_m':clearance,'one_raw_interval_max_path_bound_m':maximum,'joint_reaches':chain})
report={'qualification':'nominal table interpolation; not measured passive finger deflection','method':'triangle inequality: sum ancestor reach times absolute joint-angle delta; covers every interpolation parameter and every OBB corner','pieces':checks,'max_step_bound_m':max(x['one_raw_interval_max_path_bound_m'] for x in checks),'reserve_budget_m':{'gripper_between_raw':.001,'arm_between_samples':.001,'feedback_drift':.001,'previous_arm_endpoint_deviation':.001,'remaining_nominal_geometry':.001},'hardware_io':False}
report.update(model_id=json.loads((assets/'manifest.json').read_text())['model_id'],geometry_sha256=hashlib.sha256((assets/'gripper-kinematics.json').read_bytes()).hexdigest())
(out/'geometry-coverage.json').write_text(json.dumps(report,indent=2));print(json.dumps({'parts':len(checks),'max_step_bound_mm':report['max_step_bound_m']*1000,'covered_vertices':sum(x['source_vertices'] for x in checks)}))
