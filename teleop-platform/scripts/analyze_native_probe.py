"""Offline wire/receiver timing comparison; never contacts a device."""
import argparse
import json
from pathlib import Path
import numpy as np
from scapy.all import rdpcap, IP, TCP

p=argparse.ArgumentParser()
p.add_argument('directory',type=Path)
a=p.parse_args()
summary=json.loads((a.directory/'summary.json').read_text())
frames=[json.loads(line) for line in (a.directory/'frames.jsonl').read_text().splitlines()]
packets=rdpcap(str(a.directory/'tcp.pcap'))
feedback=[p for p in packets if TCP in p and p[TCP].sport==30004 and bytes(p[TCP].payload)]
data=[dict(time_s=float(p.time), seq=p[TCP].seq, length=len(bytes(p[TCP].payload))) for p in feedback]
gaps=np.diff([p['time_s'] for p in data])*1000
receive_gaps=np.diff([f['received_ns'] for f in frames])/1e6
commands=[dict(command=e['command'].split('(')[0],sent_ns=e['sent_ns'],
    reply_ms=(e.get('reply_ns',e.get('failed_ns',e['sent_ns']))-e['sent_ns'])/1e6,
    acknowledged=e.get('acknowledged',False)) for e in summary['commands']]
wire_commands=[dict(time_s=float(p.time),source=str(p[IP].src), text=bytes(p[TCP].payload).decode('ascii','replace'))
    for p in packets if TCP in p and (p[TCP].sport==29999 or p[TCP].dport==29999) and bytes(p[TCP].payload)]
result=dict(frames=len(frames), wire_feedback_segments=len(data),
    largest_wire_gaps=[dict(gap_ms=float(gaps[i]),before=data[i],after=data[i+1]) for i in np.argsort(gaps)[-5:]],
    largest_receive_gap_ms=float(max(receive_gaps)), commands=commands,
    wire_commands=wire_commands,
    joint_peak_to_peak_deg=np.ptp([f['status']['joint_state']['q_actual'] for f in frames],axis=0).tolist())
(a.directory/'timing_analysis.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='wire_commands'},indent=2))
