"""Measure the real reader's decode cost from one archived frame, without IO.

Never starts a reader or opens a device connection. Output is diagnosis only.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import statistics
import time

from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.dobot_transport import FeedbackFrame


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_jsonl',type=Path)
    parser.add_argument('--count',type=int,default=1000)
    args=parser.parse_args()
    if not 1<=args.count<=10000:raise ValueError('Count outside diagnostic bound')
    with args.raw_jsonl.open(encoding='utf8') as source:row=json.loads(source.readline())
    raw=base64.b64decode(row['raw_b64'])
    reader=FeedbackReader('offline-no-device')
    reader.latest=FeedbackFrame(raw,b'',time.perf_counter_ns())
    reader.connected=True;reader.generation='offline';reader.sequence=1
    wall=[];cpu=[]
    for _ in range(args.count):
        begin=time.perf_counter_ns();start_cpu=time.process_time_ns()
        result=reader.snapshot()
        cpu.append((time.process_time_ns()-start_cpu)/1e6)
        wall.append((time.perf_counter_ns()-begin)/1e6)
        assert result['status'] is not None and len(result['status']['joint_state']['q_actual'])==6
    def summary(values):
        ordered=sorted(values)
        return dict(median=statistics.median(values),p99=ordered[min(len(ordered)-1,int(.99*len(ordered)))],max=max(values))
    print(json.dumps(dict(count=args.count,wall_ms=summary(wall),cpu_ms=summary(cpu),
        frame_sha256=hashlib.sha256(raw).hexdigest(),clock='local process monotonic and CPU',
        hardware_connections=0,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())))


if __name__=='__main__':main()
