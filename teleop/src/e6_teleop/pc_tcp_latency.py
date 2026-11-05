"""Read-only check of the explicitly configured Linux PC peer route."""
import ipaddress
import json
import subprocess
import sys


def minimum_rto_ms(route):
    metrics=route.get('metrics',{})
    if isinstance(metrics,list):metrics={k:v for m in metrics for k,v in m.items()}
    value=metrics.get('rto_min')
    if isinstance(value,(int,float)):return float(value)
    if isinstance(value,str) and value.endswith('ms'):return float(value[:-2])
    if isinstance(value,str) and value.endswith('s'):return float(value[:-1])*1000
    return None


def configuration(report):
    return {k:report.get(k) for k in ('platform','requested_min_rto_ms','effective_min_rto_ms','interface','preview_renderer')}


def inspect_pc_route(sock,rto_ms):
    report=dict(platform=sys.platform,requested_min_rto_ms=rto_ms,effective_min_rto_ms=None,
                interface=None,preview_renderer=None,application_retry=False,deadline_changed=False)
    if rto_ms is None:return report
    if sys.platform!='linux' or rto_ms!=20:raise ValueError('PC minimum RTO requires Linux and explicit 20 ms')
    peer=str(ipaddress.IPv4Address(sock.getpeername()[0]))
    source=str(ipaddress.IPv4Address(sock.getsockname()[0]))
    if not ipaddress.ip_address(peer).is_private:raise ValueError('Private PC SSH peer required')
    def command(*args):
        return json.loads(subprocess.run(['ip','-j',*args],capture_output=True,text=True,
                                         timeout=3,check=True).stdout)
    exact=command('route','show','exact',peer+'/32')
    selected=command('route','get',peer)
    if (len(exact)!=1 or len(selected)!=1 or str(exact[0].get('protocol'))!='242'
            or exact[0].get('dev')!='eth0' or minimum_rto_ms(exact[0])!=20
            or selected[0].get('dev')!='eth0' or selected[0].get('prefsrc')!=source):
        raise ValueError('Explicit owned PC peer route with 20 ms floor is missing')
    report.update(peer=peer,source=source,interface='eth0',effective_min_rto_ms=20,
                  route=exact[0],scope='temporary Linux PC host route; all PC TCP flows to this X5 peer')
    return report
