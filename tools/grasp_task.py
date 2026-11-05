"""Bounded fixed-station grasp cycles. Default validates files offline.

Linux --execute uses one Dashboard and one CAN owner; no enabling, recovery,
disable or shutdown. --simulate never imports the hardware backend.
"""
from __future__ import annotations
import argparse,hashlib,json,math,os,threading,time
from datetime import datetime,timezone
from pathlib import Path
from .e6_zero import E6Error
from .e6_grasp_vertical import validate_plan as validate_vertical
from .e6_complete_tool_park import validate_plan as validate_park

LEGS=('descend_high','descend_mid','descend_low','probe_lift','peak_lift','replace','retreat_low','retreat_high')
Z_PAIRS=((280,240),(240,210),(210,190),(190,200),(200,220),(220,190),(190,235),(235,280))
ROOT=Path(__file__).resolve().parents[1]
SOURCES=('tools/grasp_task.py','tools/grasp_task_plan.py','tools/grasp_task_scene.py','tools/grasp_task_hardware.py',
         'tools/grasp_task_vision.py','tools/grasp_geometry.py','tools/e6_model.py',
         'tools/e6_zero.py','tools/probe_e6_readonly.py','tools/e6_step.py',
         'tools/e6_calibration_view.py','tools/e6_observation_move.py',
         'tools/e6_grasp_vertical.py','tools/e6_complete_tool_park.py',
         'tools/grip_monitor.py','omnipicker/tools/grasp_can.py',
         'omnipicker/tools/verify_can_control.py','config/e6_standby_pose.json',
         'config/e6_fixed_grasp_profile.json','research/sources/me6_robot_ec201c40.urdf')

def utc():return datetime.now(timezone.utc).isoformat()
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def finite_vector(v,n):return isinstance(v,list) and len(v)==n and all(type(x) in (int,float) and math.isfinite(x) for x in v)
def source_manifest():return {name:digest(ROOT/name) for name in SOURCES}

def validate_execution_review(bundle,now=None):
    review=bundle['review']
    if review['kind']!='live_preview':raise E6Error('Historical/offline examples cannot execute hardware')
    now=now or datetime.now(timezone.utc)
    for field in ('created_utc','clearance_checked_utc'):
        try:age=(now-datetime.fromisoformat(review[field])).total_seconds()
        except (KeyError,ValueError,TypeError) as exc:raise E6Error('Missing dated live review') from exc
        if not 0<=age<=1800:raise E6Error('Live task preview or physical clearance review is older than 30 minutes')
    if review.get('clearance_checked') is not True or not review.get('clearance_note'):
        raise E6Error('Require a reviewed current vertical and fold-in corridor, cables and personnel clearance')
    if review.get('source_sha256')!=source_manifest():raise E6Error('Runtime source hashes changed after preview')
    if review.get('sampling')!='controller_fk':raise E6Error('Require newly sampled controller FK')

def validate_bundle(bundle):
    if bundle.get('schema')!='e6_fixed_grasp_task_v1' or bundle.get('review',{}).get('kind') not in ('offline_example','live_preview'):
        raise E6Error('Require a supported, explicitly typed task bundle')
    if bundle.get('scope')!='fixed_blue_bottle_same_location':raise E6Error('Only fixed-station original-location cycles are supported')
    if bundle.get('force_raw')!=64 or bundle.get('opening_percent')!=60 or bundle.get('closing_percent')!=10:
        raise E6Error('Task must use the tested bounded gripper targets and raw limits')
    if not finite_vector(bundle.get('target_base_mm'),3) or not finite_vector(bundle.get('hover_q'),6):raise E6Error('Invalid task geometry')
    profile=json.loads((ROOT/'config/e6_fixed_grasp_profile.json').read_text(encoding='utf-8'))
    if math.dist(bundle['target_base_mm'],profile['target_base_mm'])>1e-6 or bundle.get('T_flange_camera_mm')!=profile['T_flange_camera_mm']:
        raise E6Error('Only the pinned historical blue-bottle geometry is supported')
    if not -23<=bundle.get('table_z_mm',0)<=-20:raise E6Error('Table height outside this reviewed station')
    previous=bundle['hover_q'];poses=[]
    if set(bundle.get('plans',{}))!=set(LEGS)|{'park','view_up','view_down'}:raise E6Error('Missing or unexpected motion legs')
    for name,(z0,z1) in zip(LEGS,Z_PAIRS):
        p=bundle['plans'][name];validate_vertical(p)
        if len(p['stages'])>16:raise E6Error('Too many stages for a bounded cycle')
        if max(abs(a-b) for a,b in zip(previous,p['origin_q']))>.03:raise E6Error('Task path is disconnected')
        end=p['stages'][-1]['samples'][-1]
        if abs(p['origin_pose'][2]-z0)>.1 or abs(end['pose'][2]-z1)>.1:raise E6Error('Changed grasp/lift/return height')
        for stage in p['stages']:
            for s in stage['samples']:
                if max(abs(a-b) for a,b in zip(s['pose'][:2],bundle['target_base_mm'][:2]))>2:
                    raise E6Error('Motion axis and target disagree')
                if not min(z0,z1)-.1<=s['pose'][2]<=max(z0,z1)+.1:raise E6Error('Vertical leg left its reviewed interval')
        previous=end['q'];poses.append(end['pose'])
    if max(abs(a-b) for a,b in zip(previous,bundle['hover_q']))>.03:raise E6Error('Repeat cycle does not close at its original hover')
    # A distinct, newly sampled 20 mm observation excursion. The grasp legs
    # retain their original Z range and all guards; no incomplete target passes.
    previous=bundle['hover_q']
    for name,z0,z1 in [('view_up',280,300),('view_down',300,280)]:
        view=bundle['plans'][name];validate_vertical(view,max_z_mm=300.01)
        if len(view['stages'])>6 or max(abs(a-b) for a,b in zip(previous,view['origin_q']))>.03:
            raise E6Error('Disconnected or oversized observation excursion')
        end=view['stages'][-1]['samples'][-1]
        if abs(view['origin_pose'][2]-z0)>.1 or abs(end['pose'][2]-z1)>.1:raise E6Error('Changed observation height')
        for stage in view['stages']:
            for sample in stage['samples']:
                if not 279.99<=sample['pose'][2]<=300.01 or max(abs(a-b) for a,b in zip(sample['pose'][:2],bundle['target_base_mm'][:2]))>2:
                    raise E6Error('Observation excursion left its reviewed axis')
        previous=end['q']
    if max(abs(a-b) for a,b in zip(previous,bundle['hover_q']))>.03:raise E6Error('Observation excursion does not return to hover')
    park=bundle['plans']['park'];validate_park(park)
    if max(abs(a-b) for a,b in zip(previous,park['origin_q']))>.03:raise E6Error('Park leg is disconnected')
    return bundle

def load_bundle(path,sha256,*,execute=False,now=None):
    path=Path(path)
    if digest(path)!=sha256:raise E6Error('Task bundle SHA mismatch')
    bundle=validate_bundle(json.loads(path.read_text(encoding='utf-8')))
    if execute:
        validate_execution_review(bundle,now)
    return bundle

class Journal:
    """Thread-safe per-event records. Hardware safety Stop must bypass I/O."""
    def __init__(self,output,mode):
        self.root=Path(output);self.root.mkdir(parents=True,exist_ok=False)
        self.lock=threading.RLock();self.mode=mode;self.phase='setup';self.cycle=0;self.sequence=0
        self.file=(self.root/'events.jsonl').open('x',encoding='utf-8')
        self.write('session',clock_semantics='Host UTC and monotonic receipt times, not exposure synchronization or a LeRobot dataset')
    def write(self,kind,**data):
        with self.lock:
            self.sequence+=1
            event=dict(sequence=self.sequence,utc=utc(),host_monotonic_ns=time.monotonic_ns(),mode=self.mode,cycle=self.cycle,phase=self.phase,kind=kind)
            if set(event)&set(data):raise ValueError('Event data overwrites journal metadata')
            event.update(data)
            self.file.write(json.dumps(event,ensure_ascii=False,allow_nan=False)+'\n');self.file.flush()
    def enter(self,phase,cycle):
        with self.lock:self.phase=phase;self.cycle=cycle;self.write('phase_enter')
    def checkpoint(self):
        with self.lock:self.file.flush();os.fsync(self.file.fileno())
    def artifact(self,name,value):
        path=self.root/name
        path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('x',encoding='utf-8') as out:json.dump(value,out,indent=2,ensure_ascii=False,allow_nan=False)
        return str(path.relative_to(self.root))
    def close(self):
        with self.lock:
            if not self.file.closed:self.file.flush();self.file.close()

class TaskRunner:
    def __init__(self,bundle,backend,journal,*,cycles=3,clock=time.monotonic):
        validate_bundle(bundle)
        if type(cycles) is not int or not 1<=cycles<=5:raise E6Error('Run only 1 to 5 bounded cycles')
        self.bundle=bundle;self.backend=backend;self.journal=journal;self.cycles=cycles;self.clock=clock;self.deadline=None
        self.report=dict(schema='e6_grasp_task_result_v1',mode=journal.mode,scope=bundle['scope'],started_utc=utc(),status='starting',requested_cycles=cycles,completed_cycles=0,phases=[],physical_success_claim=False)
    def step(self,name,cycle,action):
        if self.deadline is not None and self.clock()>self.deadline:raise E6Error('Cycle time budget exceeded')
        self.journal.enter(name,cycle);self.backend.guard()
        one=dict(cycle=cycle,phase=name,status='starting',started_utc=utc());self.report['phases'].append(one)
        try:
            result=action();self.backend.guard()
            if self.deadline is not None and self.clock()>self.deadline:raise E6Error('Cycle time budget exceeded')
            one.update(status='completed',result=result,ended_utc=utc());self.journal.write('phase_completed',result=result);self.journal.checkpoint()
            return result
        except BaseException as exc:
            one.update(status='failed',error=repr(exc),ended_utc=utc());raise
    def check(self,result):
        if result.get('status')!='passed':raise E6Error('Visual gate did not pass: '+json.dumps(result))
        return result
    def run(self):
        b=self.backend;p=self.bundle['plans'];j=self.journal
        try:
            self.step('preflight',0,lambda:b.preflight(self.bundle))
            for cycle in range(1,self.cycles+1):
                self.deadline=self.clock()+600
                self.step('open',cycle,b.open)
                self.step('view_up',cycle,lambda:b.move('view_up',p['view_up']))
                before=self.step('observe_hover',cycle,lambda:b.observe('hover'))
                self.step('locate_target',cycle,lambda:self.check(b.verify_target(before,self.bundle)))
                self.step('view_down',cycle,lambda:b.move('view_down',p['view_down']))
                for name in LEGS[:3]:self.step(name,cycle,lambda name=name:b.move(name,p[name]))
                contact=self.step('grasp',cycle,b.grasp)
                if type(contact.get('position_raw')) is not int or not 6<=contact['position_raw']<=149:raise E6Error('Invalid measured contact result')
                base=self.step('observe_contact',cycle,lambda:b.observe('contact'))
                self.step('check_contact_image',cycle,lambda:self.check(b.verify_contact(base)))
                self.step('probe_lift',cycle,lambda:b.move('probe_lift',p['probe_lift']))
                lifted=self.step('observe_probe',cycle,lambda:b.observe('probe'))
                self.step('verify_lift',cycle,lambda:self.check(b.verify_lift(base,lifted)))
                self.step('peak_lift',cycle,lambda:b.move('peak_lift',p['peak_lift']))
                self.step('observe_peak',cycle,lambda:b.observe('peak'))
                self.step('replace',cycle,lambda:b.move('replace',p['replace']))
                self.step('release',cycle,b.release)
                for name in LEGS[6:]:self.step(name,cycle,lambda name=name:b.move(name,p[name]))
                self.step('release_view_up',cycle,lambda:b.move('release_view_up',p['view_up']))
                after=self.step('observe_released_hover',cycle,lambda:b.observe('hover'))
                self.step('verify_release',cycle,lambda:self.check(b.verify_release(before,after,self.bundle)))
                self.step('release_view_down',cycle,lambda:b.move('release_view_down',p['view_down']))
                self.step('verify_cycle_end',cycle,lambda:b.verify_pose(self.bundle['hover_q']))
                self.report['completed_cycles']+=1;j.write('cycle_completed');j.checkpoint()
            self.deadline=self.clock()+300
            self.step('park',self.cycles,lambda:b.move('park',p['park']))
            self.step('verify_park',self.cycles,lambda:b.verify_pose(p['park']['target_q']))
            self.report.update(status='completed',physical_success_claim=j.mode=='hardware')
        except BaseException as exc:
            self.report.update(status='aborted',error=repr(exc),failed_phase=j.phase)
            try:self.report['stop']=b.stop_once()
            except BaseException as stop_exc:self.report['stop_error']=repr(stop_exc)
        finally:
            try:self.report['cleanup']=b.close()
            except BaseException as exc:
                self.report.update(status='cleanup_unconfirmed',cleanup_error=repr(exc),physical_success_claim=False)
                try:self.report['stop']=b.stop_once()
                except BaseException as stop_exc:self.report['stop_error']=repr(stop_exc)
            self.report.update(ended_utc=utc(),end_policy='Keep enabled; no automatic release on failure, recovery, disable or poweroff')
            try:j.artifact('summary.json',self.report)
            except BaseException as exc:self.report.update(status='recording_unconfirmed',summary_write_error=repr(exc),physical_success_claim=False)
            try:j.close()
            except BaseException as exc:self.report.update(status='recording_unconfirmed',journal_close_error=repr(exc),physical_success_claim=False)
        return self.report

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--cycles',type=int,default=3);p.add_argument('--output',type=Path)
    mode=p.add_mutually_exclusive_group();mode.add_argument('--simulate',action='store_true');mode.add_argument('--execute',action='store_true')
    a=p.parse_args();bundle=load_bundle(a.bundle,a.sha256,execute=a.execute)
    if not 1<=a.cycles<=5:p.error('Cycles must be 1..5')
    if not a.simulate and not a.execute:
        print(json.dumps(dict(status='offline_validated',review=bundle['review'],cycles=a.cycles,legs=list(bundle['plans']),hardware_connections=0),indent=2));return 0
    if a.output is None:p.error('A new output directory is required')
    if a.execute:
        from .grasp_task_hardware import HardwareBackend
        backend_type=HardwareBackend
    else:
        from .grasp_task_sim import SimulatedBackend
        backend_type=SimulatedBackend
    journal=Journal(a.output,'hardware' if a.execute else 'simulation')
    journal.artifact('bundle.json',bundle)
    backend=backend_type(journal)
    report=TaskRunner(bundle,backend,journal,cycles=a.cycles).run()
    print(json.dumps({k:report.get(k) for k in ['status','mode','requested_cycles','completed_cycles','failed_phase','error','physical_success_claim']},indent=2))
    return int(report['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
