"""Bounded per-case supervision for isolated, synthetic control experiments."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

from qualification_cleanup import Cleanup


def validate_isolation(addresses,routes):
    loopback=False
    for link in addresses:
        if link['ifname']=='lo' and link['link_type']=='loopback':
            loopback='UP' in link['flags']
            continue
        # Linux may create these inert tunnel templates in a fresh netns.
        # They have no address, no UP flag and no route to an external device.
        if ((link['ifname'],link['link_type']) not in {('tunl0','ipip'),('sit0','sit')} or
                'UP' in link['flags'] or link.get('addr_info') or link['operstate']!='DOWN'):
            raise RuntimeError('Non-loopback connectivity in qualification namespace')
    if not loopback or any(route.get('dev')!='lo' for route in routes):
        raise RuntimeError('Qualification requires only loopback routes')


def run_case(argv,receipt,timeout,grace=15):
    receipt=Path(receipt)
    result=dict(argv=argv,timeout_s=timeout,timed_out=False,error=None,returncode=None,
                start_ns=time.perf_counter_ns(),physical_io=False)
    cleanup=Cleanup(receipt.with_suffix('.cleanup.json'))
    child=None
    try:
        child=subprocess.Popen(argv,start_new_session=True)
        result['pid']=child.pid
        try:child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:result['timed_out']=True
    except Exception:result['error']=traceback.format_exc()
    finally:
        if child is not None:
            def signal_wait(sig,seconds):
                if child.poll() is None:
                    os.killpg(child.pid,sig)
                    child.wait(timeout=seconds)
            if child.poll() is None:
                cleanup.run('interrupt',lambda:signal_wait(signal.SIGINT,grace))
            if child.poll() is None:
                cleanup.run('terminate',lambda:signal_wait(signal.SIGTERM,5))
            if child.poll() is None:
                cleanup.run('kill',lambda:signal_wait(signal.SIGKILL,3))
            result['returncode']=child.poll()
        result.update(end_ns=time.perf_counter_ns(),cleanup_failures=cleanup.failures)
        receipt.write_text(json.dumps(result,indent=2),encoding='utf8')
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--worker-root',type=Path,required=True)
    a=p.parse_args()
    addresses=json.loads(subprocess.check_output(['ip','-j','address','show']))
    routes=[]
    for family in ('-4','-6'):
        routes+=json.loads(subprocess.check_output(['ip',family,'-j','route','show','table','all']))
    validate_isolation(addresses,routes)
    (a.root/'network-isolation.json').write_text(json.dumps(dict(addresses=addresses,routes=routes)))
    cases=[('medium-joint','joint',.5,False),('high-joint','joint',1,False),
           ('medium-alt','alternating',.5,False),('high-alt','alternating',1,False),
           ('high-combined','combined',1,False),('high-tool','combined',1,True)]
    all_results=[]
    for tag,mode,scale,tool in cases:
        argv=[sys.executable,str(a.root/'qualify_writer.py'),'--root',str(a.root),
              '--worker-root',str(a.worker_root),'--tag',tag,'--paced','--seconds','36',
              '--mode',mode,'--scale',str(scale),'--sdk-feedback']
        if tool:argv.append('--tool')
        result=run_case(argv,a.root/(tag+'-supervisor.json'),130)
        # The worker has its own session. Verify exact process identity before
        # cleaning it after a harness crash; never kill by name or stale PID.
        proof=a.root/tag/'worker-process.json'
        if proof.exists():
            worker=json.loads(proof.read_text())
            stat=Path('/proc',str(worker['pid']),'stat')
            try:fields=stat.read_text().rsplit(')',1)[1].split()
            except FileNotFoundError:fields=None
            if fields is not None and fields[0]!='Z' and fields[19]==worker['start_ticks']:
                from e6_phosphobot.jazzy_host import finish_owned_group
                if int(fields[2])!=worker['pgid'] or worker['pgid']!=worker['pid']:
                    raise RuntimeError('Owned worker process-group identity changed')
                result['worker_cleanup_needed']=True
                finish_owned_group(worker['pgid'])
        all_results.append(result)
        (a.root/'case-results.json').write_text(json.dumps(all_results,indent=2))
    return int(any(x['returncode']!=0 or x['timed_out'] or x['error'] or
                   x['cleanup_failures'] or x.get('worker_cleanup_needed') for x in all_results))


if __name__=='__main__':raise SystemExit(main())
