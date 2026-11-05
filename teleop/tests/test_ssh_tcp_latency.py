"""Temporary peer route ownership, exact readback and newly opened SSH sockets."""
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from e6_observer.ssh_tcp_latency import SshTcpLatency,minimum_rto_ms
from e6_teleop.tunnel import Tunnel


class Route(SshTcpLatency):
    def __init__(self):
        super().__init__(20,'192.168.1.7 43210 192.168.1.13 22')
        self.routes=[];self.calls=[]

    def command(self,*args):
        self.calls.append(args)
        if args[:3]==('-j','route','show'):value=self.routes
        elif args[:3]==('-j','route','get'):value=[dict(dev='wlan0',prefsrc='192.168.1.13')]
        elif args[:2]==('route','add'):
            self.routes=[dict(dst='192.168.1.7',dev='wlan0',protocol=242,prefsrc='192.168.1.13',metrics=[dict(rto_min=20)])];value=[]
        elif args[:2]==('route','del'):self.routes=[];value=[]
        else:raise AssertionError(args)
        return SimpleNamespace(stdout=json.dumps(value))


def test_debian_metric_and_exact_restore():
    setting=Route()
    with setting:
        assert setting.report['applied'] and minimum_rto_ms(setting.routes[0])==20
        assert not setting.report['application_retry'] and not setting.report['deadline_changed']
    assert setting.routes==[] and setting.report['restored']


def test_refuse_existing_route_and_external_changes():
    setting=Route();setting.routes=[dict(dev='wlan0')]
    with pytest.raises(ValueError,match='Existing'):setting.__enter__()
    assert not any(v[:2]==('route','add') for v in setting.calls)
    setting.routes=[];setting.__enter__();setting.routes[0]['protocol']=241
    with pytest.raises(ValueError,match='outside'):setting.__exit__()
    assert setting.routes[0]['protocol']==241


@pytest.mark.parametrize('metrics',[{'rto_min':20},[{'rto_min':20}],{'rto_min':'20ms'},{'rto_min':'0.02s'}])
def test_metrics_representation(metrics):assert minimum_rto_ms(dict(metrics=metrics))==20


def test_only_owned_control_and_accepted_parameter():
    with pytest.raises(ValueError,match='owned'):Tunnel('x5',ssh_rto_min_ms=20)
    with pytest.raises(ValueError,match='acceptance'):
        Tunnel('x5',remote_root='/owned',control_stage='live',control_role='operator',
               acceptance_profile={'ssh_rto_min_ms':None},ssh_rto_min_ms=20)
