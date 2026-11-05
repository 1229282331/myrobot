"""Offline X5 loopback TCP reassembly with the installed websockets parser.

Never compare the PC and X5 absolute clocks. Capture begins before handshake.
"""
from collections import defaultdict,Counter
import gzip,hashlib,json,socket,sys
from pathlib import Path
import dpkt
from websockets.frames import Frame,Opcode
from analyze_input_relay import stats


def frames(data, start):
    position=start
    def read_exact(size):
        nonlocal position
        if position+size>len(data):raise EOFError('Partial last frame')
        result=data[position:position+size];position+=size
        if False:yield
        return result
    while position<len(data):
        begin=position
        generator=Frame.parse(read_exact,mask=True,max_size=16384)
        try:
            next(generator)
            raise RuntimeError('Unexpected parser suspension with complete byte buffer')
        except StopIteration as done:
            yield begin,position,done.value
        except EOFError:
            return


def analyze(base, audit):
    base=Path(base);clock=json.loads((base/'clock.json').read_text())
    streams=defaultdict(list);counts=Counter();wire={};connections=[]
    with (base/'wire.pcap').open('rb') as source:
        for ts,packet in dpkt.pcap.Reader(source):
            ip=dpkt.ethernet.Ethernet(packet).data
            if not isinstance(ip,dpkt.ip.IP) or not isinstance(ip.data,dpkt.tcp.TCP):continue
            tcp=ip.data
            if tcp.dport!=8776 or not tcp.data:continue
            streams[(socket.inet_ntoa(ip.src),tcp.sport)].append((tcp.seq,bytes(tcp.data),round(float(ts)*1e9)))
    for peer,segments in streams.items():
        first=segments[0][0];buf=bytearray();spans=[]
        for seq,data,ts in sorted((((seq-first+2**31)%2**32)-2**31,data,t) for seq,data,t in segments):
            if seq>len(buf):raise ValueError('Missing TCP bytes')
            overlap=max(0,len(buf)-seq)
            if overlap:
                size=min(overlap,len(data));counts['overlap_bytes']+=size
                if bytes(buf[seq:seq+size])!=data[:size]:raise ValueError('Conflicting TCP retransmission')
            if len(data)>overlap:
                spans.append((len(buf),len(buf)+len(data)-overlap,ts));buf.extend(data[overlap:])
        if not buf.startswith(b'GET '):raise ValueError('Capture began after HTTP handshake')
        header_end=buf.index(b'\r\n\r\n')+4
        connections.append(dict(peer=peer,segments=len(segments),bytes=len(buf)))
        last_end=header_end
        for start,end,frame in frames(bytes(buf),header_end):
            last_end=end;counts['websocket_frames']+=1
            if frame.opcode in (Opcode.PING,Opcode.PONG,Opcode.CLOSE):continue
            if frame.opcode!=Opcode.TEXT or not frame.fin:raise ValueError('Fragmented/nontext input unsupported by this offline analyzer')
            data=json.loads(frame.data)
            if 'seq' not in data or 'client_sample_ms' not in data:continue
            timestamp=max(t for a,b,t in spans if b>start and a<end)
            key=(data['seq'],data['client_sample_ms'])
            if key in wire:raise ValueError('Ambiguous message key across connections')
            wire[key]=dict(data=data,wire_wall_ns=timestamp,peer=peer)
        counts['trailing_bytes']+=len(buf)-last_end
    pairs=[clock['before'],clock['after']]
    lo=min(p['mono_before_ns']-p['wall_ns'] for p in pairs)-1000
    hi=max(p['mono_after_ns']-p['wall_ns'] for p in pairs)+1000
    inputs=[];matches=[]
    with gzip.open(audit,'rt',encoding='utf8') as source:
        for line in source:
            event=json.loads(line)
            if event['event']!='input':continue
            data=event['data'];key=(data['seq'],data['client_sample_ms'])
            if key not in wire:continue  # Outside capture window, excluded explicitly.
            hit=wire[key]
            if data!=hit['data']:raise ValueError('Wire payload differs from recorded input')
            row=dict(sequence=data['seq'],wire_wall_ns=hit['wire_wall_ns'],x5_dequeue_ns=event['at_ns'],
                lower_ms=(event['at_ns']-hit['wire_wall_ns']-hi)/1e6,
                upper_ms=(event['at_ns']-hit['wire_wall_ns']-lo)/1e6)
            matches.append(row)
            if data['op']=='intent':inputs.append(dict(**row,data=data))
    groups=[];active=[]
    for e in inputs:
        if e['data']['intent']['deadman']:active.append(e)
        elif active:groups.append(active+[e]);active=[]
    incomplete = (dict(first_sequence=active[0]['sequence'],last_sequence=active[-1]['sequence'],
                       matched_inputs=len(active),reason='Capture ended before release; excluded from complete-run integrals')
                  if active else None)
    runs=[]
    for group in groups:
        area=dict(browser=0.,x5_loopback=0.,x5_dequeue=0.)
        phases=[]
        for a,b in zip(group,group[1:]):
            velocity=a['data']['intent']['linear_m_s'][2]*1000
            dt=dict(browser=(b['data']['client_sample_ms']-a['data']['client_sample_ms'])/1000,
                    x5_loopback=(b['wire_wall_ns']-a['wire_wall_ns'])/1e9,
                    x5_dequeue=(b['x5_dequeue_ns']-a['x5_dequeue_ns'])/1e9)
            for k,v in dt.items():area[k]+=velocity*v
            if not phases or phases[-1]['velocity_mm_s']!=velocity:
                phases.append(dict(sequence=a['sequence'],velocity_mm_s=velocity,duration_s={k:0. for k in dt}))
            for k,v in dt.items():phases[-1]['duration_s'][k]+=v
        runs.append(dict(first_sequence=group[0]['sequence'],last_sequence=group[-1]['sequence'],
            input_count=len(group)-1,phases=phases,z_area_mm=area,
            active_receive_upper_ms=stats([e['upper_ms'] for e in group]),
            loopback_to_dequeue_area_mm=area['x5_dequeue']-area['x5_loopback']))
    return dict(connections=connections,counts=dict(counts),matched_inputs=len(matches),incomplete_active_group=incomplete,
                clock_bracket_width_ms=(hi-lo)/1e6,
                dequeue_delay_lower_ms=stats([e['lower_ms'] for e in matches]),
                dequeue_delay_upper_ms=stats([e['upper_ms'] for e in matches]),runs=runs,
                matches=matches,
                method='dpkt TCP reassembly + official websockets Frame.parse; full decoded input object matches gateway log. X5-local wall/monotonic brackets include 1us quantization, assume no unobserved clock step. Loopback begins after SSH decryption; before-loopback residual includes PC tunnel, SSH, wireless and sshd. No absolute PC/X5 clock subtraction.',
                sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [base/'wire.pcap',base/'clock.json',Path(audit)]})


if __name__=='__main__':
    result=analyze(*sys.argv[1:3])
    Path(sys.argv[3]).write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({k:([{x:y for x,y in r.items() if x!='phases'} for r in v] if k=='runs' else v)
                      for k,v in result.items() if k not in ('matches','sha256')},indent=2))
