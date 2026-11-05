"""Temporary peer-only SSH TCP route metric; no application retransmission."""
import ipaddress
import json
import os
import subprocess


def minimum_rto_ms(route):
    """iproute2 emits metrics as a list on Debian and a dict on some releases."""
    metrics=route.get('metrics',{})
    if isinstance(metrics,list):
        metrics={k:v for item in metrics if isinstance(item,dict) for k,v in item.items()}
    value=metrics.get('rto_min') if isinstance(metrics,dict) else None
    if isinstance(value,(int,float)):return float(value)
    if isinstance(value,str):
        if value.endswith('ms'):return float(value[:-2])
        if value.endswith('s'):return float(value[:-1])*1000
    return None


class SshTcpLatency:
    def __init__(self,rto_ms=None,connection=None):
        if rto_ms not in (None,20):raise ValueError('SSH minimum RTO must be the explicit 20 ms option')
        self.rto_ms=rto_ms;self.changed=False;self.applied=None
        self.connection=os.environ.get('SSH_CONNECTION','') if connection is None else connection
        self.report=dict(requested_min_rto_ms=rto_ms,applied=False,
                         scope='temporary host route to this SSH peer; all X5 TCP flows to that peer',
                         application_retry=False,deadline_changed=False)

    def command(self,*args):
        return subprocess.run(['ip',*args],capture_output=True,text=True,timeout=3,check=True)

    def exact(self):
        return json.loads(self.command('-j','route','show','exact',self.prefix).stdout)

    def __enter__(self):
        if self.rto_ms is None:return self
        fields=self.connection.split()
        if len(fields)!=4:raise ValueError('Owned SSH_CONNECTION required for peer route')
        self.peer=str(ipaddress.IPv4Address(fields[0]));self.source=str(ipaddress.IPv4Address(fields[2]))
        if not ipaddress.ip_address(self.peer).is_private or self.peer==self.source:raise ValueError('Private distinct SSH peer required')
        if not all(x.isdigit() and 0<int(x)<65536 for x in (fields[1],fields[3])):raise ValueError('Invalid SSH endpoint ports')
        self.prefix=self.peer+'/32'
        if self.exact():raise ValueError('Existing peer host route; refuse to replace it')
        selected=json.loads(self.command('-j','route','get',self.peer).stdout)
        if len(selected)!=1 or selected[0].get('dev')!='wlan0' or selected[0].get('prefsrc')!=self.source or selected[0].get('gateway'):
            raise ValueError('Expected directly attached wlan0 SSH peer')
        self.report.update(peer=self.peer,source=self.source,original_lookup=selected)
        try:
            self.command('route','add',self.prefix,'dev','wlan0','src',self.source,
                         'proto','242','rto_min','20ms')
            self.changed=True;self.applied=self.exact()
            if len(self.applied)!=1 or self.applied[0].get('dev')!='wlan0' or minimum_rto_ms(self.applied[0])!=self.rto_ms:
                raise ValueError('Peer TCP metric readback failed')
            self.report.update(applied=True,route_readback=self.applied)
        except BaseException:
            self.__exit__();raise
        return self

    def __exit__(self,*args):
        if self.changed:
            if self.exact()!=self.applied:raise ValueError('Peer route changed outside this session; refuse to overwrite it')
            self.command('route','del',self.prefix,'dev','wlan0','proto','242')
            if self.exact():raise ValueError('Peer TCP route restoration failed')
            self.changed=False;self.report['restored']=True
            print('Session SSH peer TCP route restored',flush=True)
