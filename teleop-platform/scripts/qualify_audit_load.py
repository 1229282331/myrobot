"""Read-only recorded-data load; no device libraries, sockets or motion APIs."""
import argparse,importlib.util,json,queue,threading,time,statistics,sys
from pathlib import Path
from types import SimpleNamespace


class Reader:
 def __init__(self):self.subs=[];self.lock=threading.Lock()
 def subscribe(self,capacity):
  s=SimpleNamespace(queue=queue.Queue(capacity),dropped=0)
  with self.lock:self.subs.append(s)
  return s
 def unsubscribe(self,s):
  with self.lock:self.subs.remove(s)
 def publish(self,item):
  with self.lock:
   for sub in self.subs:
    try:sub.queue.put_nowait(item)
    except queue.Full:sub.dropped+=1


def main():
 sys.setswitchinterval(.001)  # Match the production gateway handoff policy.
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--variant',choices=['baseline','candidate'],required=True);ap.add_argument('--tag',required=True);ap.add_argument('--seconds',type=float,default=12);a=ap.parse_args()
 if a.variant=='baseline':
  original=a.root/'baseline/audit.py'
  if not original.exists():original=a.root/'baseline/source/audit.py'
  spec=importlib.util.spec_from_file_location('baseline_audit',original);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);Audit=module.Audit
 else:from e6_phosphobot.audit import Audit
 out=a.root/('audit-load-'+a.tag);out.mkdir(exist_ok=False);reader=Reader();writers=[Audit(out/label,reader) for label in ('continuous','episode')]
 cycle=json.loads((a.root/'fixtures.json').read_text())[0]['cycle'];halt=threading.Event();raw_count=[0];event_count=[0];ticks=[]
 def raw():
  target=time.perf_counter()
  while not halt.is_set():
   raw_count[0]+=1;reader.publish(('test',raw_count[0],SimpleNamespace(received_ns=time.perf_counter_ns(),prefix=b'',raw=b'x'*1440)))
   target+=.008;halt.wait(max(0,target-time.perf_counter()))
 def events():
  target=time.perf_counter()
  while not halt.is_set():
   event=dict(cycle,at_ns=time.perf_counter_ns());event_count[0]+=1
   for w in writers:w.put(event)
   target+=.05;halt.wait(max(0,target-time.perf_counter()))
 threads=[threading.Thread(target=f) for f in (raw,events)]
 for t in threads:t.start()
 begin=time.perf_counter();target=begin
 while time.perf_counter()-begin<a.seconds:
  target+=.005;time.sleep(max(0,target-time.perf_counter()));now=time.perf_counter();ticks.append(max(0,now-target)*1000)
  # Representative local pipe JSON decoding competes with the journals.
  json.loads(json.dumps(cycle['state']['feedback']))
 halt.set()
 for t in threads:t.join()
 for w in writers:w.close()
 ticks.sort();result=dict(variant=a.variant,seconds=a.seconds,raw=raw_count[0],events=event_count[0],
  scheduler_lateness_ms=dict(p50=statistics.median(ticks),p95=ticks[int(.95*len(ticks))],max=max(ticks)),writers=[dict(counts=w.counts,error=w.error) for w in writers])
 for w in writers:
  assert not w.error,(a.variant,w.error)
  assert w.counts==dict(raw=raw_count[0],events=event_count[0])
 (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
