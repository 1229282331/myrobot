import json
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from e6_teleop.pc_tcp_latency import inspect_pc_route,configuration


class Wire:
    def getpeername(self):return ('192.168.1.13',22)
    def getsockname(self):return ('172.24.219.228',40000)


@pytest.mark.parametrize('violation',[None,'missing','wrong-floor','wrong-owner','wrong-source'])
def test_pc_route_readback_never_accepts_missing_unowned_or_changed_configuration(violation):
    route=dict(dst='192.168.1.13',dev='eth0',protocol=242,metrics=[dict(rto_min=20)])
    lookup=dict(dev='eth0',prefsrc='172.24.219.228')
    if violation=='wrong-floor':route['metrics']=[dict(rto_min=300)]
    if violation=='wrong-owner':route['protocol']=241
    if violation=='wrong-source':lookup['prefsrc']='172.24.219.2'
    calls=[]
    def run(args,**kwargs):
        calls.append(args)
        value=([lookup] if args[2:4]==['route','get'] else [] if violation=='missing' else [route])
        return SimpleNamespace(stdout=json.dumps(value))
    with patch.dict('os.environ',LIBGL_ALWAYS_SOFTWARE='1',MESA_LOADER_DRIVER_OVERRIDE='llvmpipe'),patch('e6_teleop.pc_tcp_latency.sys.platform','linux'),patch('e6_teleop.pc_tcp_latency.subprocess.run',run):
        if violation:
            with pytest.raises(ValueError):inspect_pc_route(Wire(),20)
        else:
            report=inspect_pc_route(Wire(),20)
            assert configuration(report)==dict(platform='linux',requested_min_rto_ms=20,effective_min_rto_ms=20,interface='eth0',preview_renderer=None)
            assert not report['application_retry'] and not report['deadline_changed']
        assert all(args[:3]==['ip','-j','route'] for args in calls)
        assert all('add' not in args and 'del' not in args for args in calls)


def test_windows_minimum_rto_is_not_modified_or_claimed():
    with patch('e6_teleop.pc_tcp_latency.sys.platform','win32'):
        assert configuration(inspect_pc_route(Wire(),None))['effective_min_rto_ms'] is None
        with pytest.raises(ValueError):inspect_pc_route(Wire(),20)


def test_split_ssh_profile_keeps_linux_route_and_windows_clock_domains_explicit():
    from e6_teleop.wsl_tunnel import split_profile
    original=dict(platform='linux',requested_min_rto_ms=20,effective_min_rto_ms=20,interface='eth0',preview_renderer=None,source='172.24.219.228')
    result=split_profile(original)
    assert result['pc_clock_platform']=='win32' and result['ssh_socket_platform']=='linux'
    assert result['linux_transport_provenance']==original
    assert original['platform']=='linux'
    for key,value in [('effective_min_rto_ms',300),('interface','wlan0'),('platform','win32')]:
        with pytest.raises(ValueError):split_profile(dict(original,**{key:value}))
