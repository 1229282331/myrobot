"""Temporary, E6-only TCP RTO route metric. Never changes retry semantics.

TCP still de-duplicates segments; this does not resend application commands.
This opt-in diagnostic refuses to replace existing host routes and restores
only the exact route it created. Explicit owned control can request 20 ms;
other values remain available only to the independent read-only diagnostic.
"""
from ipaddress import IPv4Address
import json
import subprocess


def command(*args):
    return subprocess.run(args,capture_output=True,text=True,timeout=3,check=True).stdout


def inspect_e6_route(ip,run=command):
    """Read the selected transport, without adding or changing any route."""
    rows=json.loads(run('ip','-j','route','get',str(IPv4Address(ip))))
    if len(rows)!=1 or rows[0].get('dev') not in ('eth0','wlan0') or not rows[0].get('prefsrc'):
        raise ValueError('E6 route has no unambiguous supported interface/source')
    row=rows[0]
    return dict(robot_ip=str(IPv4Address(ip)),interface=row['dev'],source=row['prefsrc'],
                gateway=row.get('gateway'),direct_ethernet=row['dev']=='eth0' and not row.get('gateway'))


class TcpLatency:
    def __init__(self,ip,rto_ms=None,run=command):
        self.ip=str(IPv4Address(ip));self.rto_ms=rto_ms;self.run=run;self.created=None
        if rto_ms is not None and (type(rto_ms) is not int or not 10<=rto_ms<=50):
            raise ValueError('Diagnostic TCP RTO minimum must be 10-50 ms')
        self.report=dict(requested_ms=rto_ms,temporary=True,changed=False,restored=False,
                         application_retries=0,scope='E6 host route, not a control deadline')

    def routes(self):
        return json.loads(self.run('ip','-j','-details','route','show','table','main','exact',self.ip+'/32'))

    def __enter__(self):
        if self.rto_ms is None:return self
        if self.routes():raise ValueError('Existing E6 host route; diagnostic refuses replacement')
        current=json.loads(self.run('ip','-j','route','get',self.ip))
        if len(current)!=1 or current[0].get('dev')!='wlan0':
            raise ValueError('Diagnostic requires a single wlan0 E6 route')
        self.report['original_route']=current[0]
        args=['ip','route','add',self.ip+'/32']
        if current[0].get('gateway'):args+=['via',current[0]['gateway']]
        args+=['dev','wlan0','proto','static']
        if current[0].get('prefsrc'):args+=['src',current[0]['prefsrc']]
        self.run(*args,'rto_min',str(self.rto_ms)+'ms')
        try:
            self.created=self.routes()
            if len(self.created)!=1:raise RuntimeError('Created E6 host route could not be verified')
            from .ssh_tcp_latency import minimum_rto_ms
            if minimum_rto_ms(self.created[0])!=self.rto_ms:raise RuntimeError('Created E6 RTO metric differs from request')
            self.report.update(changed=True,applied_route=self.created)
        except BaseException:
            # __exit__ is not called when __enter__ fails. The successful add
            # belongs to this attempt; remove it before propagating the error.
            self.run('ip','route','del',self.ip+'/32','dev','wlan0','proto','static')
            self.created=None
            raise
        return self

    def __exit__(self,*args):
        if self.created is None:return
        current=self.routes()
        if not current:
            self.report['restored']=True;return
        if current!=self.created:raise RuntimeError('E6 host route changed externally; refuse deletion')
        self.run('ip','route','del',self.ip+'/32','dev','wlan0','proto','static')
        if self.routes():raise RuntimeError('Temporary E6 host route removal not verified')
        self.created=None;self.report['restored']=True
        print('Temporary E6 TCP route metric removed',flush=True)
