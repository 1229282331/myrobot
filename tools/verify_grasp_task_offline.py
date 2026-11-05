"""Repeatable local verification; blocks sockets and hardware subprocesses."""
import argparse,contextlib,io,json,shutil,sys,unittest
from pathlib import Path
from unittest.mock import patch
from . import grasp_task as task,grasp_task_plan as plan,grasp_task_vision as vision

TESTS=('tests.test_grasp_task','tests.test_grasp_task_plan','tests.test_grasp_task_vision','tests.test_grasp_task_hardware','tests.test_grasp_task_scene',
       'tests.test_e6_calibration_view','tests.test_e6_complete_tool_park','tests.test_e6_grasp_vertical',
       'tests.test_grip_monitor','omnipicker.tests.test_grasp_can')

def observation(name):
    r=json.loads((plan.HISTORY/name/'report.json').read_text(encoding='utf-8'))
    return dict(image=str(plan.HISTORY/name/'0008_color.png'),pose=json.loads((plan.HISTORY/(name+'_observation.json')).read_text(encoding='utf-8'))['after']['pose'],calibration=r['calibration'])

def cli(function,argv):
    stream=io.StringIO()
    with patch.object(sys,'argv',['offline_verification',*argv]),contextlib.redirect_stdout(stream):code=function()
    return dict(exit_code=code,output=stream.getvalue())

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    result=dict(mode='offline_software_validation',physical_success_claim=False,hardware_connections=0,started_utc=task.utc())
    with contextlib.ExitStack() as stack:
        for name in ('socket.create_connection','socket.socket','subprocess.Popen','subprocess.run'):
            stack.enter_context(patch(name,side_effect=AssertionError('Verification prohibits hardware access')))
        with (a.output/'unit_tests.txt').open('x',encoding='utf-8') as log:
            tests=unittest.defaultTestLoader.loadTestsFromNames(TESTS)
            tested=unittest.TextTestRunner(stream=log,verbosity=2).run(tests)
        result['tests']=dict(run=tested.testsRun,failures=len(tested.failures),errors=len(tested.errors),passed=tested.wasSuccessful())
        if not tested.wasSuccessful():raise task.E6Error('Offline tests failed; see unit_tests.txt')
        result['example_cli']=cli(plan.main,['--example','--output',str(a.output/'example')])
        bundle=a.output/'example/bundle.json';sha=task.digest(bundle);common=['--bundle',str(bundle),'--sha256',sha]
        result['default_cli']=cli(task.main,common)
        result['help_cli']=None
        stream=io.StringIO()
        with patch.object(sys,'argv',['grasp_task','--help']),contextlib.redirect_stdout(stream):
            try:task.main()
            except SystemExit as exc:result['help_cli']=dict(exit_code=exc.code,output=stream.getvalue())
        result['simulations']={}
        for count in (3,5):
            result['simulations'][str(count)]=cli(task.main,[*common,'--simulate','--cycles',str(count),'--output',str(a.output/f'simulation_{count}')])
            if result['simulations'][str(count)]['exit_code']!=0:raise task.E6Error('Simulation did not complete')
        try:cli(task.main,[*common,'--execute','--output',str(a.output/'forbidden_hardware')])
        except task.E6Error as exc:result['historical_execute_rejected']=str(exc)
        else:raise AssertionError('Historical example execution must be rejected')
        before,contact,probe,after=[observation(n) for n in ('open60_view_2310','force64_contact190_2336','lift10_view_2339','final_hover280_2356')]
        profile=plan.read_profile()
        regression=dict(target_before=vision.target(before,profile),contact=vision.contact(contact),
                        probe_lift=vision.lift(contact,probe),original_location_release=vision.released(before,after,profile),
                        provenance={n:dict(image=o['image'],image_sha256=task.digest(o['image'])) for n,o in zip(('before','contact','probe','after'),(before,contact,probe,after))},
                        scope='Historical-image offline regression; no new hardware or clearance measurement')
        (a.output/'vision_regression.json').write_text(json.dumps(regression,indent=2),encoding='utf-8')
    source_names=(*task.SOURCES,'tools/grasp_task_sim.py','tools/verify_grasp_task_offline.py',
                  *(name.replace('.','/')+'.py' for name in TESTS))
    hashes={}
    for name in source_names:
        target=a.output/'sources'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(task.ROOT/name,target)
        hashes[name]=task.digest(target)
    (a.output/'source_manifest.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    result.update(status='passed',ended_utc=task.utc(),source_sha256=hashes,bundle_sha256=sha)
    (a.output/'verification_summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('status','mode','tests','hardware_connections','physical_success_claim','bundle_sha256')},indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
