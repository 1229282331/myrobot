"""Exercise route ownership and restoration without any system mutation."""
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.tcp_latency import TcpLatency,inspect_e6_route


class Routes:
    def __init__(self):self.route=[];self.calls=[];self.bad_verify=False
    def __call__(self,*args):
        self.calls.append(args)
        if 'get' in args:return json.dumps([dict(dst='192.168.1.4',dev='wlan0',prefsrc='192.168.1.13')])
        if 'show' in args:
            if self.bad_verify and self.route:raise RuntimeError('route inspection failed')
            return json.dumps(self.route)
        if 'add' in args:self.route=[dict(dst='192.168.1.4',dev='wlan0',protocol='static',metrics=[dict(rto_min=20)])]
        if 'del' in args:self.route=[]
        return ''


def test_route_restored_after_diagnostic_failure():
    system=Routes();ctx=TcpLatency('192.168.1.4',20,system)
    with pytest.raises(TimeoutError):
        with ctx:
            assert ctx.report['changed'] and system.route
            raise TimeoutError('read-only query failed')
    assert not system.route and ctx.report['restored']


def test_existing_or_externally_modified_routes_are_not_deleted():
    system=Routes();system.route=[dict(dst='192.168.1.4',dev='eth0')]
    with pytest.raises(ValueError):TcpLatency('192.168.1.4',20,system).__enter__()
    assert not any('add' in c or 'del' in c for c in system.calls)
    system.route=[];ctx=TcpLatency('192.168.1.4',20,system);ctx.__enter__()
    system.route[0]['dev']='eth0'
    with pytest.raises(RuntimeError,match='externally'):ctx.__exit__()
    assert system.route[0]['dev']=='eth0' and not any('del' in c for c in system.calls)


def test_failed_enter_removes_its_successful_add():
    system=Routes();system.bad_verify=True
    with pytest.raises(RuntimeError,match='inspection'):TcpLatency('192.168.1.4',20,system).__enter__()
    assert not system.route and any('del' in c for c in system.calls)


def test_disabled_setting_does_not_inspect_or_modify_routes():
    system=Routes()
    with TcpLatency('192.168.1.4',None,system):pass
    assert not system.calls


@pytest.mark.parametrize('interface,source,direct',[('eth0','192.168.1.14',True),('wlan0','192.168.1.13',False)])
def test_actual_e6_transport_readback_has_no_route_mutation(interface,source,direct):
    calls=[]
    def run(*args):
        calls.append(args);return json.dumps([dict(dev=interface,prefsrc=source)])
    profile=inspect_e6_route('192.168.1.4',run)
    assert profile==dict(robot_ip='192.168.1.4',interface=interface,source=source,gateway=None,direct_ethernet=direct)
    assert calls==[('ip','-j','route','get','192.168.1.4')]


@pytest.mark.parametrize('rows',[[],[{}],[dict(dev='eth0')],[dict(dev='usb0',prefsrc='192.168.1.14')]])
def test_unknown_route_cannot_be_advertised_as_direct_e6(rows):
    with pytest.raises(ValueError):inspect_e6_route('192.168.1.4',lambda *args:json.dumps(rows))


@pytest.mark.parametrize('metrics',[[],[{'rto_min':10}],[{'rto_min':'0.2s'}]])
def test_route_metric_mismatch_is_rejected_and_own_add_rolled_back(metrics):
    system=Routes()
    def run(*args):
        value=system(*args)
        if 'add' in args:system.route[0]['metrics']=metrics
        return value
    with pytest.raises(RuntimeError,match='differs'):TcpLatency('192.168.1.4',20,run).__enter__()
    assert not system.route and any('del' in c for c in system.calls)


def test_control_e6_route_opt_in_is_startup_fixed_and_matches_operator_acceptance():
    from e6_teleop.tunnel import Tunnel
    for options in ({},{'remote_root':'/owned'},{'remote_root':'/owned','control_stage':'live','e6_rto_min_ms':10}):
        with pytest.raises(ValueError,match='owned'):Tunnel('x5',**dict({'e6_rto_min_ms':20},**options))
    with pytest.raises(ValueError,match='differs'):
        Tunnel('x5',remote_root='/owned',control_stage='live',control_role='operator',acceptance_profile={'e6_rto_min_ms':None},e6_rto_min_ms=20)
    assert Tunnel('x5',remote_root='/owned',control_stage='shadow',e6_rto_min_ms=20).e6_rto_min_ms==20
