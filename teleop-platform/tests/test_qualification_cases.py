import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest

pytestmark=pytest.mark.skipif(os.name=='nt',reason='Owned POSIX process group supervisor')
scripts=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(scripts))
spec=importlib.util.spec_from_file_location('qualification_cases',scripts/'qualification_cases.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_nonzero_exit_is_recorded_and_next_case_can_run(tmp_path):
    failed=module.run_case([sys.executable,'-c','raise SystemExit(7)'],tmp_path/'failed.json',3)
    clean=module.run_case([sys.executable,'-c','pass'],tmp_path/'clean.json',3)
    assert failed['returncode']==7 and not failed['timed_out']
    assert clean['returncode']==0 and not clean['error']
    assert json.loads((tmp_path/'failed.json').read_text())['returncode']==7


def test_timeout_stays_failed_even_when_child_handles_interrupt_cleanly(tmp_path):
    script=tmp_path/'child.py'
    script.write_text('import signal,time\nsignal.signal(signal.SIGINT,lambda *a:exit(0))\n'
                      'print("ready",flush=True)\nwhile True:time.sleep(.01)\n')
    result=module.run_case([sys.executable,str(script)],tmp_path/'receipt.json',.4,grace=2)
    assert result['timed_out'] and result['returncode']==0
    assert not result['cleanup_failures']
    assert json.loads((tmp_path/'receipt.json').read_text())['timed_out']


@pytest.mark.parametrize('case',['inert','ethernet','tunnel_up','address','external_route','lo_down'])
def test_default_tunnels_are_inert_and_external_connectivity_is_rejected(case):
    links=[dict(ifname='lo',link_type='loopback',flags=['LOOPBACK','UP']),
           dict(ifname='tunl0',link_type='ipip',flags=['NOARP'],addr_info=[],operstate='DOWN'),
           dict(ifname='sit0',link_type='sit',flags=['NOARP'],addr_info=[],operstate='DOWN')]
    routes=[dict(dev='lo',dst='127.0.0.0/8')]
    if case=='ethernet':links.append(dict(ifname='eth0',link_type='ether',flags=[]))
    elif case=='tunnel_up':links[1]['flags'].append('UP')
    elif case=='address':links[1]['addr_info']=[dict(local='192.168.1.1')]
    elif case=='external_route':routes.append(dict(dev='tunl0',dst='default'))
    elif case=='lo_down':links[0]['flags'].remove('UP')
    if case=='inert':module.validate_isolation(links,routes)
    else:
        with pytest.raises(RuntimeError):module.validate_isolation(links,routes)
