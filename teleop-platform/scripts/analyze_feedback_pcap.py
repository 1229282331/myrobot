"""Offline TCP reassembly with dpkt 1.9.8; never repairs a live feedback stream.

Only E6:30004 inbound payload is inspected. Gaps split independent runs and
conflicting retransmissions are reported, rather than guessed or discarded.
"""
import argparse
import hashlib
import json
import socket
import struct
from collections import defaultdict, Counter
from pathlib import Path
import dpkt


def analyze(path):
    flows=defaultdict(list)
    with path.open('rb') as f:
        for stamp,wire in dpkt.pcap.Reader(f):
            ethernet=dpkt.ethernet.Ethernet(wire)
            ip=ethernet.data
            if not isinstance(ip,dpkt.ip.IP) or not isinstance(ip.data,dpkt.tcp.TCP):continue
            tcp=ip.data
            if socket.inet_ntoa(ip.src)!='192.168.1.4' or tcp.sport!=30004 or not tcp.data:continue
            flows[(socket.inet_ntoa(ip.dst),tcp.dport)].append((tcp.seq,float(stamp),bytes(tcp.data)))
    reports=[]
    for key,packets in flows.items():
        origin=packets[0][0]
        packets=sorted((((s-origin+2**31)%2**32)-2**31,t,d) for s,t,d in packets)
        runs=[];current=None;gaps=[];conflicts=[]
        for seq,stamp,data in packets:
            if current is None or seq>current['end']:
                if current is not None:gaps.append(dict(start=current['end'],end=seq))
                current=dict(start=seq,end=seq,bytes=bytearray(),segments=[]);runs.append(current)
            if seq<current['end']:
                overlap=min(current['end']-seq,len(data));offset=seq-current['start']
                if current['bytes'][offset:offset+overlap]!=data[:overlap]:
                    conflicts.append(dict(seq=seq,overlap=overlap,stamp=stamp))
                data=data[overlap:];seq+=overlap
            if not data:continue
            current['segments'].append((len(current['bytes']),stamp,len(data)))
            current['bytes'].extend(data);current['end']+=len(data)
        run_reports=[]
        for run in runs:
            payload=bytes(run['bytes']);heads=[];offset=0
            while True:
                offset=payload.find(struct.pack('<Q',1440),offset)
                if offset<0:break
                if payload[offset+48:offset+56]==struct.pack('<Q',0x0123456789abcdef):heads.append(offset)
                offset+=1
            diffs=Counter(b-a for a,b in zip(heads,heads[1:]))
            unusual=[]
            for a,b in zip(heads,heads[1:]):
                if b-a not in (1440,1448):
                    stamp=next((t for start,t,n in run['segments'] if start<=a<start+n),None)
                    unusual.append(dict(offset=a,next_header_offset=b,distance=b-a,pcap_wall_time=stamp,
                        prefix_hex=payload[a:b].hex() if b-a<256 else None))
            run_reports.append(dict(bytes=len(payload),sha256=hashlib.sha256(payload).hexdigest(),
                headers=len(heads),header_distances=dict(diffs),unusual=unusual,
                first_packet_wall_time=run['segments'][0][1],last_packet_wall_time=run['segments'][-1][1]))
        reports.append(dict(peer=key,payload_packets=len(packets),tcp_gaps=gaps,
            conflicting_retransmissions=conflicts,runs=run_reports))
    return dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),dpkt_version=dpkt.__version__,
        flows=reports,scope='Passive eth0 ingress capture. No live resynchronization or guessed actual state. '
            'Kernel capture-drop counts are in the separate tcpdump log.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('pcap',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=analyze(a.pcap)
    a.output.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
