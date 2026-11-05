"""Planner sampling under synthetic read-only IK/FK; never a controller test."""
import copy,json,re,unittest
from unittest.mock import patch
import numpy as np
from tools import grasp_task_plan as plan
from tools.e6_zero import E6Error

class FakeKinematics:
    """Interpolate historical end states for protocol and closure checks only."""
    def __init__(self):
        p=plan.read_profile();self.hover=p['reviewed_hover_q'];self.pose=p['reviewed_hover_pose'];self.commands=[]
        names=('descent240_plan_2313','descent210_plan_2320','descent190_plan_2323','lift10_plan_2337','lift_remaining20_plan_2340','retreat235_plan_2350')
        top=self.hover.copy();top[1]+=5;top[2]-=5
        pairs=[(280,self.hover),(300,top)]
        for name in names:
            b=json.loads((plan.HISTORY/(name+'.json')).read_text());s=b['stages'][-1]['samples'][-1];pairs.append((round(s['pose'][2]),s['q']))
        pairs.sort();self.z=np.array([z for z,_ in pairs]);self.q=np.array([q for _,q in pairs])
    def q_at(self,z):return [float(np.interp(z,self.z,self.q[:,i])) for i in range(6)]
    def command(self,cmd):
        self.commands.append(cmd)
        if cmd=='RobotMode()':return '5'
        if cmd=='GetErrorID()':return '[]'
        if cmd=='GetAngle()':return ','.join(map(str,self.hover))
        if cmd.startswith('GetPose'):return ','.join(map(str,self.pose))
        v=[float(x) for x in re.findall(r'-?\d+\.\d+',cmd)[:6]]
        if cmd.startswith('InverseKin'):result=self.q_at(v[2])
        elif cmd.startswith('PositiveKin'):
            # Park moves beyond the vertical inverse model's elbow range.
            if v[2]>self.hover[2]+.1 and v[1]>self.hover[1]+.1:
                f=(v[1]-self.hover[1])/(-60-self.hover[1]);result=[a+(b-a)*f for a,b in zip(self.pose,[-6.3359,-189.3302,395.1109,-135.3707,6.4895,-163.0334])]
            else:
                result=self.pose.copy();result[2]=float(np.interp(v[1],self.q[:,1],self.z))
        else:raise AssertionError('Write attempted by planner: '+cmd)
        return ','.join(f'{x:.6f}' for x in result)

class PlanTests(unittest.TestCase):
    def setUp(self):
        for name in ('socket.create_connection','socket.socket','subprocess.Popen','subprocess.run'):
            p=patch(name,side_effect=AssertionError('Planner tests are offline'));p.start();self.addCleanup(p.stop)
    def test_live_sampling_closes_cycle_and_never_sends_write_commands(self):
        profile=plan.read_profile();robot=FakeKinematics();obs=dict(pose=profile['reviewed_hover_pose'])
        bundle,queries=plan.live_bundle(robot,profile,obs)
        self.assertEqual(bundle['hover_q'],bundle['plans']['retreat_high']['stages'][-1]['target_q'])
        self.assertEqual(bundle['review']['sampling'],'controller_fk');self.assertFalse(bundle['review']['clearance_checked'])
        self.assertGreater(len(queries),700)
        self.assertTrue(all(x.startswith(('RobotMode','GetAngle','GetErrorID','GetPose','PositiveKin','InverseKin')) for x in robot.commands))
        self.assertTrue(all(len(s['samples'])==9 for p in bundle['plans'].values() for s in p['stages']))
    def test_changed_hover_state_is_rejected_without_ik(self):
        robot=FakeKinematics();robot.hover=robot.hover.copy();robot.hover[0]+=.1
        with self.assertRaises(E6Error):plan.live_bundle(robot,plan.read_profile(),dict(pose=robot.pose))
        self.assertFalse(any(c.startswith('InverseKin') for c in robot.commands))
    def test_changed_scene_pose_is_rejected_without_ik(self):
        p=plan.read_profile();robot=FakeKinematics();pose=p['reviewed_hover_pose'].copy();pose[0]+=1
        with self.assertRaises(E6Error):plan.live_bundle(robot,p,dict(pose=pose))
        self.assertFalse(any(c.startswith('InverseKin') for c in robot.commands))
    def test_ik_fk_disagreement_is_rejected(self):
        p=plan.read_profile();robot=FakeKinematics();original=robot.command
        def reply(cmd):
            if cmd.startswith('InverseKin'):return '-60,-80,65,-60,90,9'
            return original(cmd)
        robot.command=reply
        with self.assertRaisesRegex(E6Error,'closure'):plan.live_bundle(robot,p,dict(pose=p['reviewed_hover_pose']))
    def test_query_allowlist_blocks_all_writes(self):
        robot=FakeKinematics();planner=plan.ReadOnlyPlanner(robot)
        with self.assertRaises(E6Error):planner.query('EnableRobot()')
        self.assertEqual(robot.commands,[])

if __name__=='__main__':unittest.main()
