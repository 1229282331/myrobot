"""Deterministic offline task backend; never opens sockets or devices."""
from .e6_zero import E6Error
from omnipicker.tools.grasp_can import ContactGate,HoldGate,ReleaseGate

class SimulatedBackend:
    def __init__(self,journal,*,failure_phase=None,failure_point='action',cancelled=lambda:False):
        self.journal=journal;self.failure_phase=failure_phase;self.failure_point=failure_point;self.cancelled=cancelled
        self.q=None;self.pose=None;self.contact_position=None;self.position=153;self.holding=False;self.stop_count=0;self.bundle=None
    def fail(self,point):
        if self.failure_phase==self.journal.phase and self.failure_point==point:raise E6Error('Injected offline failure at '+self.journal.phase)
    def guard(self):
        if self.cancelled():raise E6Error('Task cancelled')
        if self.holding:HoldGate(self.contact_position).update(self.feedback(2,self.position))
    def feedback(self,state,pos):return dict(fault=0,state=state,position_raw=pos,simulated=True)
    def preflight(self,bundle):
        self.fail('action');self.bundle=bundle;self.q=bundle['hover_q'].copy();self.pose=bundle['plans']['descend_high']['origin_pose'].copy()
        self.journal.write('robot_state',robot_mode=5,joints_deg=self.q,pose=self.pose,simulated=True)
        return dict(status='passed',hardware_connections=0,simulated=True)
    def open(self):
        self.fail('action');self.holding=False;self.position=153
        self.journal.write('gripper_feedback',feedback=self.feedback(0,153),simulated=True)
        return dict(position_raw=153,stable_frames=20,simulated=True)
    def grasp(self):
        self.fail('action');gate=ContactGate(self.position);contact=None
        for pos in list(range(149,102,-4))+[103]*22:
            f=self.feedback(1 if pos!=103 else 2,pos);contact=gate.update(f) or contact
            self.journal.write('gripper_feedback',feedback=f,simulated=True)
        if contact is None:raise E6Error('Simulated contact did not establish')
        self.contact_position=contact['position_raw'];self.position=self.contact_position;self.holding=True
        return dict(contact,simulated=True)
    def release(self):
        self.fail('action');gate=ReleaseGate(self.contact_position)
        for i,pos in enumerate([self.position,self.position+10,140]+[153]*20):
            f=self.feedback(2 if i==0 else 0,pos);gate.update(f,i*.05)
            self.journal.write('gripper_feedback',feedback=f,simulated=True)
        self.position=153;self.holding=False
        return dict(position_raw=153,stable_frames=20,simulated=True)
    def move(self,name,plan):
        self.fail('action')
        if max(abs(a-b) for a,b in zip(self.q,plan['origin_q']))>.15:raise E6Error('Simulated start mismatch')
        for i,stage in enumerate(plan['stages']):
            self.journal.write('robot_command',command='simulated MovJ',target_q=stage['target_q'],stage=i,simulated=True)
            for sample in stage['samples']:
                self.q=sample['q'].copy();self.pose=sample['pose'].copy();self.fail('motion_sample');self.guard()
                self.journal.write('robot_state',robot_mode=7,joints_deg=self.q,pose=self.pose,simulated=True)
        return dict(status='reached',stages=len(plan['stages']),simulated=True)
    def observe(self,kind):
        self.fail('action');self.guard();self.journal.write('observation',view=kind,pose=self.pose,simulated=True)
        return dict(simulated=True,view=kind,pose=self.pose.copy())
    def verify_target(self,obs,bundle):self.fail('visual');return dict(status='passed',simulated=True,scope='Synthetic gate, not physical image verification')
    def verify_contact(self,obs):self.fail('visual');return dict(status='passed',simulated=True)
    def verify_lift(self,before,after):self.fail('visual');return dict(status='passed',simulated=True)
    def verify_release(self,before,after,bundle):self.fail('visual');return dict(status='passed',simulated=True)
    def verify_pose(self,target):
        self.fail('action');self.guard()
        if max(abs(a-b) for a,b in zip(self.q,target))>.05:raise E6Error('Simulated endpoint mismatch')
        return dict(status='passed',joints_deg=self.q,simulated=True)
    def stop_once(self):
        if self.stop_count==0:self.stop_count=1
        try:self.journal.write('stop',attempts=self.stop_count,simulated=True)
        except OSError:pass
        return dict(attempts=self.stop_count,automatic_release=False,simulated=True)
    def close(self):return dict(status='closed',hardware_connections=0,simulated=True)
