"""Bounded single CAN stage: opening or closing with measured contact gates.

No arm motion. Defaults to offline preview. A stalled state is only accepted
after closing progress and stable nonzero position; it is not proof of a grasp.
"""
import argparse,hashlib,json,math,signal,sys,time
from pathlib import Path
from datetime import datetime,timezone
try:
    from . import verify_can_control as can
except ImportError:
    import verify_can_control as can

LIMITS={'velocity':10,'force':32,'acceleration':25,'deceleration':25}

def bounded_limits(force_raw=32):
    if type(force_raw) is not int or force_raw not in (32,64):raise ValueError('Require a tested force limit, raw32 or raw64')
    return dict(LIMITS,force=force_raw)

class ContactGate:
    def __init__(self,start_position):
        if type(start_position) is not int or not 38<=start_position<=153:
            raise ValueError('Require measured 15..60 percent opening')
        self.start=start_position;self.previous=start_position;self.progress=0;self.contact=[]
    def update(self,f):
        if f.get('fault')!=0 or f.get('state') not in (0,1,2):raise can.CheckFailed('Fault, drop or invalid state')
        pos=f['position_raw']
        if not 0<=pos<=255 or pos>self.start+2 or abs(pos-self.previous)>12:
            raise can.CheckFailed('Unexpected position excursion')
        self.progress=max(self.progress,self.start-pos);self.previous=pos
        if pos<=2 and f['state']==0:raise can.CheckFailed('Closed empty; no verified contact')
        if f['state']==2:
            if self.progress<4 or pos<6:raise can.CheckFailed('Stall without measured closing progress or nonzero contact')
            self.contact.append(pos);self.contact=self.contact[-20:]
            if len(self.contact)>=20:
                if max(self.contact)-min(self.contact)>2:raise can.CheckFailed('Contact position unstable')
                return {'position_raw':round(sum(self.contact)/len(self.contact)),
                        'stable_frames':len(self.contact),'scope':'Stable resistance while closing; image verification required before lift'}
        else:self.contact=[]
        return None

class HoldGate:
    def __init__(self,contact_position):
        if type(contact_position) is not int or not 6<=contact_position<=149:
            raise ValueError('Invalid contact position')
        self.position=contact_position
    def update(self,f):
        if f.get('fault')!=0 or f.get('state')!=2 or abs(f['position_raw']-self.position)>2:
            raise can.CheckFailed('Grasp holding feedback changed; prevent or stop arm motion')

class ReleaseGate:
    """Known contact may be reported briefly before an opening takes effect."""
    def __init__(self,position):
        HoldGate(position);self.position=position;self.moving=False
    def update(self,f,elapsed):
        if f.get('fault')!=0 or f.get('state') not in (0,1,2) or f['position_raw']<self.position-2:
            raise can.CheckFailed('Release fault, drop or unexpected closing')
        if f['state']==2:
            if self.moving or elapsed>.2 or abs(f['position_raw']-self.position)>2:
                raise can.CheckFailed('Release remained stalled beyond its known initial contact')
        else:self.moving=True
        if elapsed>1.5 and f['position_raw']<self.position+4:
            raise can.CheckFailed('Release produced no opening progress')

def run_stage(channel,action,target,report,*,start_position=None,seconds=6,force_raw=32,
              stopped=lambda:False,clock=time.monotonic,sleep=time.sleep):
    if action not in ('open','grasp','release') or not 1<=seconds<=8:
        raise ValueError('Invalid bounded stage')
    if action in ('open','release') and not 15<=target<=60 or action=='grasp' and not 0<=target<=10:
        raise ValueError('Invalid stage target')
    command=can.encode_command(target,bounded_limits(force_raw),bounds=(0,60));report['feedback']=[]
    gate=ContactGate(start_position) if action=='grasp' else None
    release=ReleaseGate(start_position) if action=='release' else None
    if release is not None and command[1]<start_position+4:raise ValueError('Release target must open away from known contact')
    begin=clock();next_send=begin;stable=0;contact=None
    while clock()-begin<seconds:
        if stopped():raise can.CheckFailed('Cancellation; no remaining targets')
        f=channel.exchange(command);f['elapsed_s']=clock()-begin;report['feedback'].append(f)
        if action in ('open','release'):
            if release is not None:release.update(f,clock()-begin)
            elif f.get('fault')!=0 or f['state'] not in (0,1):raise can.CheckFailed('Opening fault/stall/drop')
            stable=stable+1 if can.arrived(f,command[1]) else 0
        elif contact is None:
            if abs(f['position_raw']-command[1])<=1 and f['state']==0:
                raise can.CheckFailed('Closing target reached without verified resistance')
            contact=gate.update(f)
            if clock()-begin>1.5 and gate.progress<4:
                raise can.CheckFailed('Closing command produced no measurable progress')
            if contact is not None:report['contact']=contact;hold=HoldGate(contact['position_raw']);stable=1
        else:hold.update(f);stable+=1
        next_send+=.05;sleep(max(0,next_send-clock()))
    if stable<20:raise can.CheckFailed('Stage did not finish with 20 stable frames')
    report['last_feedback']=report['feedback'][-1];report['command_hex']=command.hex()
    return contact if action=='grasp' else report['last_feedback']

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--action',choices=['open','grasp','release'],required=True)
    p.add_argument('--target',type=float,required=True);p.add_argument('--start-position',type=int)
    p.add_argument('--seconds',type=float,default=6);p.add_argument('--output',type=Path);p.add_argument('--execute',action='store_true')
    p.add_argument('--force-raw',type=int,choices=[32,64],default=32)
    a=p.parse_args()
    if not math.isfinite(a.target) or not 1<=a.seconds<=8 or (a.action in ('open','release') and not 15<=a.target<=60) or (a.action=='grasp' and not 0<=a.target<=10):p.error('Invalid stage')
    if a.action=='grasp':ContactGate(a.start_position)
    if a.action=='release':ReleaseGate(a.start_position)
    preview=dict(action=a.action,target_percent=a.target,limits_raw=bounded_limits(a.force_raw),seconds=a.seconds,
                 command_hex=can.encode_command(a.target,bounded_limits(a.force_raw),bounds=(0,60)).hex(),arm_motion_commands=0)
    if not a.execute:print(json.dumps(preview,indent=2));return 0
    if sys.platform!='linux' or a.output is None:p.error('Linux and new output required')
    import fcntl
    report=dict(preview,started_at_utc=datetime.now(timezone.utc).isoformat(),events=[],status='starting',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),can_source_sha256=hashlib.sha256(Path(can.__file__).read_bytes()).hexdigest())
    cancelled=[False];channel=None;configured=False
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,lambda *_:cancelled.__setitem__(0,True))
    with a.output.open('x',encoding='utf-8') as f,open('/tmp/omnipicker-can1.lock','a') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if Path('/root/omnipicker-can/STOP').exists():raise can.CheckFailed('Stop file present')
            report['identity']=can.validate_usb_interface('can1');report['before']=can.ip_snapshot('can1')
            if 'UP' in report['before']['flags']:raise can.CheckFailed('CAN already in use')
            configured=True;can.configure('can1','type','can','bitrate','1000000','sample-point','0.8','listen-only','off');can.configure('can1','up')
            report['initialized']=can.ip_snapshot('can1')
            if report['initialized'].get('linkinfo',{}).get('info_data',{}).get('bittiming',{}).get('bitrate')!=1000000:raise can.CheckFailed('Bitrate mismatch')
            channel=can.CanChannel('can1',1,.3,report,allow_stalled=a.action in ('grasp','release'))
            report['result']=run_stage(channel,a.action,a.target,report,start_position=a.start_position,seconds=a.seconds,force_raw=a.force_raw,stopped=lambda:cancelled[0] or Path('/root/omnipicker-can/STOP').exists())
            report['status']='completed'
        except BaseException as exc:report.update(status='aborted',error=repr(exc))
        finally:
            if channel:channel.close()
            if configured:
                try:report['before_down']=can.ip_snapshot('can1');can.configure('can1','down');report['after']=can.ip_snapshot('can1')
                except BaseException as exc:report.update(status='cleanup_unconfirmed',cleanup_error=repr(exc))
            report['ended_at_utc']=datetime.now(timezone.utc).isoformat();report['motor_state']='24V remains powered; CAN DOWN is not motor disable'
            json.dump(report,f,indent=2)
    print(json.dumps({k:report.get(k) for k in ['status','error','result','source_sha256','can_source_sha256']},indent=2))
    return int(report['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
