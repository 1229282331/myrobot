"""Prepare fixed-station cycles: --example is entirely offline.

--live-preview explicitly reads an already enabled hover via controller IK/FK;
never enables, moves, configures CAN, clears errors or captures a camera frame.
It requires a recent stationary observation. The resulting plan still requires
current physical corridor review before --execute is permitted.
"""
from __future__ import annotations
import argparse,copy,json,math,time
from datetime import datetime,timezone
from pathlib import Path
from .grasp_task import ROOT,LEGS,Z_PAIRS,E6Error,utc,digest,source_manifest,validate_bundle
from .e6_model import arm_origins
from .e6_calibration_view import rotation_angle
from .e6_zero import read_state,validate_joints
from . import grasp_task_vision as vision

PROFILE=ROOT/'config/e6_fixed_grasp_profile.json'
HISTORY=ROOT/'docs/evidence/first_grasp_20261003'

def read_profile():return json.loads(PROFILE.read_text(encoding='utf-8'))

def base_bundle(profile,kind):
    keys=('scope','target_base_mm','T_flange_camera_mm','table_z_mm','force_raw','opening_percent','closing_percent','camera')
    bundle={k:copy.deepcopy(profile[k]) for k in keys}
    bundle.update(schema='e6_fixed_grasp_task_v1',hover_q=profile['reviewed_hover_q'].copy(),plans={},
                  review=dict(kind=kind,created_utc=utc(),clearance_checked=False,source_sha256=source_manifest()),
                  limits=dict(speed_percent=1,acceleration_percent=1,max_segment_deg=3,max_cycles=5),
                  geometry_status=profile['geometry_status'])
    return bundle

def example_bundle():
    """Synthetic interpolated FK data for software checks, never hardware."""
    profile=read_profile();b=base_bundle(profile,'offline_example')
    names=('descent240_plan_2313','descent210_plan_2320','descent190_plan_2323','lift10_plan_2337',
           'lift_remaining20_plan_2340','return190_plan_2345','retreat235_plan_2350','retreat280_plan_2354')
    previous=b['hover_q'].copy();op=profile['reviewed_hover_pose'].copy()
    for name,old,(z0,z1) in zip(LEGS,names,Z_PAIRS):
        history=json.loads((HISTORY/(old+'.json')).read_text(encoding='utf-8'))
        goal=history['stages'][-1]['target_q'].copy()
        if name=='retreat_high':goal=b['hover_q'].copy()
        count=math.ceil(max(abs(x-y) for x,y in zip(previous,goal))/2.9)
        p=dict(status='sampled_vertical_grasp',origin_q=previous.copy(),origin_pose=op.copy(),
               origin_arm_origins_mm=arm_origins(previous),stages=[],sampling='synthetic_linear_pose_not_controller_fk')
        oq=previous.copy();start=previous.copy()
        for n in range(1,count+1):
            end=[x+(y-x)*n/count for x,y in zip(oq,goal)]
            samples=[]
            for i in range(9):
                q=[x+(y-x)*i/8 for x,y in zip(start,end)]
                pose=op.copy();pose[2]=z0+(z1-z0)*((n-1)+i/8)/count
                samples.append(dict(q=q,pose=pose,arm_origins_mm=arm_origins(q)))
            p['stages'].append(dict(index=n-1,start_q=start,target_q=end,samples=samples));start=end
        b['plans'][name]=p;previous=goal;op=p['stages'][-1]['samples'][-1]['pose'].copy()
    park=json.loads((ROOT/'docs/evidence/park_complete_tool_20261003/park_plan_0004.json').read_text(encoding='utf-8'))
    b['plans']['park']=park
    top=b['hover_q'].copy();top[1]+=5;top[2]-=5
    for name,start,end,z0,z1 in [('view_up',b['hover_q'],top,280,300),('view_down',top,b['hover_q'],300,280)]:
        p=dict(status='sampled_vertical_grasp',origin_q=start.copy(),origin_pose=profile['reviewed_hover_pose'].copy(),origin_arm_origins_mm=arm_origins(start),stages=[],sampling='synthetic_linear_pose_not_controller_fk')
        p['origin_pose'][2]=z0;previous=start.copy()
        for n in range(1,4):
            goal=[a+(z-a)*n/3 for a,z in zip(start,end)];samples=[]
            for i in range(9):
                q=[a+(z-a)*i/8 for a,z in zip(previous,goal)];pose=p['origin_pose'].copy();pose[2]=z0+(z1-z0)*((n-1)+i/8)/3
                samples.append(dict(q=q,pose=pose,arm_origins_mm=arm_origins(q)))
            p['stages'].append(dict(index=n-1,start_q=previous,target_q=goal,samples=samples));previous=goal
        b['plans'][name]=p
    b['review'].update(sampling='synthetic_vertical_with_historical_park',physical_success_claim=False,
                       warning='Offline example uses interpolated poses; never a reusable hardware path')
    return validate_bundle(b)

class ReadOnlyPlanner:
    def __init__(self,robot):self.robot=robot;self.queries=[]
    def query(self,cmd):
        if not (cmd.startswith('PositiveKin(') or cmd.startswith('InverseKin(')):
            raise E6Error('Planner permits only IK/FK reads')
        payload=self.robot.command(cmd);self.queries.append(dict(command=cmd,payload=payload))
        v=[float(x) for x in payload.split(',')]
        if len(v)!=6 or not all(math.isfinite(x) for x in v):raise E6Error('Invalid kinematic reply')
        return v
    def fk(self,q):
        validate_joints(q)
        return self.query('PositiveKin('+','.join(f'{x:.6f}' for x in q)+',user=0,tool=0)')
    def ik(self,pose,near):
        cmd='InverseKin('+','.join(f'{x:.6f}' for x in pose)+',user=0,tool=0,useJointNear=1,jointNear={'
        result=self.query(cmd+','.join(f'{x:.6f}' for x in near)+'})');validate_joints(result)
        return result
    def stage(self,start,end,index):
        samples=[]
        for i in range(9):
            q=[a+(z-a)*i/8 for a,z in zip(start,end)]
            samples.append(dict(q=q,pose=self.fk(q),arm_origins_mm=arm_origins(q)))
        return dict(index=index,start_q=start.copy(),target_q=end.copy(),samples=samples)
    def plan(self,origin_q,targets,status):
        oq=origin_q.copy();p=dict(status=status,origin_q=oq,origin_pose=self.fk(oq),origin_arm_origins_mm=arm_origins(oq),stages=[])
        start=oq
        for goal in targets:
            distance=max(abs(x-y) for x,y in zip(start,goal))
            if distance<1e-5:raise E6Error('Degenerate motion target')
            count=math.ceil(distance/2.9);anchor=start.copy()
            for n in range(1,count+1):
                end=[a+(z-a)*n/count for a,z in zip(anchor,goal)]
                p['stages'].append(self.stage(start,end,len(p['stages'])));start=end
        return p

def stationary(robot,expected):
    started=time.monotonic();s=read_state(robot);pose=[float(x) for x in robot.command('GetPose(user=0,tool=0)').split(',')]
    if time.monotonic()-started>1 or s.mode!=5 or max(abs(a-b) for a,b in zip(s.joints,expected))>.03:
        raise E6Error('Require enabled stationary robot at reviewed hover; no automatic positioning')
    if len(pose)!=6 or not all(math.isfinite(x) for x in pose):raise E6Error('Invalid hover pose')
    return dict(mode=s.mode,q=list(s.joints),pose=pose)

def scene_input(path,profile,now=None,*,allow_clipped_for_view_planning=False):
    """Only read supplied files; do not capture or operate hardware."""
    obs=json.loads(Path(path).read_text(encoding='utf-8'));run=Path(obs['source_run'])
    report_path=run/'report.json';r=json.loads(report_path.read_text(encoding='utf-8'))
    if digest(report_path)!=obs['camera_report_sha256']:raise E6Error('Scene report hash differs')
    now=now or datetime.now(timezone.utc);age=(now-datetime.fromisoformat(r['ended_at_utc'])).total_seconds()
    if obs.get('status')!='static_scene' or not 0<=age<=180 or r.get('status')!='completed' or any(r.get(k) for k in ('cleanup_errors','processing_errors','receiver_errors')):
        raise E6Error('Require fresh clean stationary scene within 180 seconds')
    for state in (obs['before'],obs['after']):
        if state['mode']!=5 or max(abs(a-b) for a,b in zip(state['q'],profile['reviewed_hover_q']))>.03:
            raise E6Error('Scene is not at the reviewed enabled hover')
    if max(abs(a-b) for a,b in zip(obs['before']['q'],obs['after']['q']))>.03:raise E6Error('Scene moved during capture')
    if r['device']['serial_number']!='AY4A1520111' or r['uvc_pts_availability']['missing_color_frames'] or r['color_decoded_frames']!=r['streams']['color']['frames'] or r['color_decoded_frames']<2:
        raise E6Error('Scene camera identity/decode/PTS failed')
    if any(s['index_gaps'] or s['index_nonmonotonic_count'] for s in r['streams'].values()):raise E6Error('Scene frame continuity failed')
    o=dict(image=str(run/'0008_color.png'),pose=obs['after']['pose'],calibration=r['calibration'])
    try:check=vision.target(o,profile)
    except E6Error as exc:
        if not allow_clipped_for_view_planning or str(exc)!='Bottle silhouette clipped by search ROI or image boundary':raise
        check=dict(status='deferred',reason=str(exc),scope='Read-only view planning only; the 300mm image must pass before descending')
    return o,dict(observation_sha256=digest(path),camera_report_sha256=digest(report_path),image_sha256=digest(o['image']),target_check=check)

def live_bundle(robot,profile,obs):
    before=stationary(robot,profile['reviewed_hover_q']);op=before['pose']
    if abs(op[2]-280)>.1 or max(abs(a-b) for a,b in zip(op[:2],profile['target_base_mm'][:2]))>1 or rotation_angle(op,profile['reviewed_hover_pose'])>.1:
        raise E6Error('Fresh hover differs from the reviewed fixed workcell')
    if max(abs(a-b) for a,b in zip(op[:3],obs['pose'][:3]))>.1:raise E6Error('New robot pose differs from scene')
    b=base_bundle(profile,'live_preview');b['hover_q']=before['q'].copy();planner=ReadOnlyPlanner(robot);previous=b['hover_q'].copy()
    for name,(z0,z1) in zip(LEGS,Z_PAIRS):
        near=previous.copy();targets=[];count=math.ceil(abs(z1-z0)/4)
        for n in range(1,count+1):
            p=op.copy();p[2]=z0+(z1-z0)*n/count
            q=b['hover_q'].copy() if name=='retreat_high' and n==count else planner.ik(p,near)
            check=planner.fk(q)
            if math.dist(check[:3],p[:3])>.1 or rotation_angle(check,p)>.1:raise E6Error('IK/FK closure failed')
            targets.append(q);near=q
        b['plans'][name]=planner.plan(previous,targets,'sampled_vertical_grasp');previous=targets[-1]
    # Newly sampled path from this run's hover; never replay a historical park plan.
    target=[previous[0],-60,105,-90,previous[4],previous[5]]
    park=planner.plan(previous,[target],'sampled_complete_tool_park')
    park.update(target_q=target,nominal_tool_radius_mm=170,table_z_mm=-22)
    b['plans']['park']=park
    top_pose=op.copy();top_pose[2]=300
    top=planner.ik(top_pose,b['hover_q']);check=planner.fk(top)
    if math.dist(check[:3],top_pose[:3])>.1 or rotation_angle(check,top_pose)>.1:raise E6Error('Observation IK/FK closure failed')
    for name,start,end in [('view_up',b['hover_q'],top),('view_down',top,b['hover_q'])]:
        b['plans'][name]=planner.plan(start,[end],'sampled_vertical_grasp')
    after=stationary(robot,b['hover_q'])
    if max(abs(a-b) for a,b in zip(before['q'],after['q']))>.03:raise E6Error('Robot moved while sampling')
    b['review'].update(created_utc=utc(),sampling='controller_fk',robot_before=before,robot_after=after)
    return validate_bundle(b),planner.queries

def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True);mode.add_argument('--example',action='store_true');mode.add_argument('--live-preview',action='store_true')
    p.add_argument('--scene-observation',type=Path);p.add_argument('--clearance-review',type=Path)
    p.add_argument('--plan-elevated-view-from-clipped-scene',action='store_true',help='Read-only preparation; full elevated target gate remains mandatory before descent')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('Output must be a new directory')
    if a.example:
        b=example_bundle();queries=[]
    else:
        if a.scene_observation is None:p.error('Fresh scene observation required')
        profile=read_profile();obs,scene=scene_input(a.scene_observation,profile,allow_clipped_for_view_planning=a.plan_elevated_view_from_clipped_scene)
        # Resolve camera paths and pin runtime/config before opening a Dashboard.
        camera=profile['camera'];runtime=Path(camera['tools_dir'])/'rdk_healthcheck.py'
        if digest(runtime)!=camera['healthcheck_sha256']:raise E6Error('Unexpected camera runtime')
        if digest(camera['sdk_config'])!=camera['sdk_config_sha256']:raise E6Error('Unexpected camera configuration')
        from .e6_observation_move import ObservationDashboard
        with ObservationDashboard('192.168.1.4',timeout=.5) as robot:b,queries=live_bundle(robot,profile,obs)
        b['review']['scene']=scene
        if a.clearance_review is not None:
            review=json.loads(a.clearance_review.read_text(encoding='utf-8'))
            if review.get('checked') is not True or not review.get('note'):raise E6Error('Incomplete physical corridor review')
            age=(datetime.now(timezone.utc)-datetime.fromisoformat(review['checked_utc'])).total_seconds()
            if not 0<=age<=1800:raise E6Error('Stale clearance review')
            b['review'].update(clearance_checked=True,clearance_checked_utc=review['checked_utc'],clearance_note=review['note'])
    a.output.mkdir(parents=True);bundle=a.output/'bundle.json';bundle.write_text(json.dumps(b,indent=2,ensure_ascii=False),encoding='utf-8')
    (a.output/'queries.json').write_text(json.dumps(queries,indent=2),encoding='utf-8')
    result=dict(status='prepared',review_kind=b['review']['kind'],bundle_sha256=digest(bundle),robot_write_commands=0,
                hardware_connections=0 if a.example else 1,segments={n:len(v['stages']) for n,v in b['plans'].items()},
                execute_ready=b['review'].get('clearance_checked',False),physical_success_claim=False)
    (a.output/'preview.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
