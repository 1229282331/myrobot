"""Retain receive/process/send chronology across every recorded normal release."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence',type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();source=a.evidence/'session/x5/audit/events.jsonl.gz'
    with gzip.open(source,'rt',encoding='utf8') as f:events=[json.loads(s) for s in f]
    groups=json.loads((a.evidence/'continuous.json').read_text(encoding='utf8'))['groups']
    result=[]
    for index,group in enumerate(groups):
        end=group['end_ns']
        release=next(e for e in events if e['event']=='input' and e['at_ns']==end)
        assert release['data']['intent']['deadman'] is False
        processed=next(e for e in events if e['event']=='input_processed' and e['received_ns']==end)
        commands=[e for e in events if e['event']=='motion_submitted' and end<=e['receipt']['sent_ns']<end+1_000_000_000]
        result.append(dict(group=index,release_received_ns=end,release_sequence=release['data']['seq'],
            processing_ns=processed['processing_ns'],processed_ns=processed['at_ns'],
            commands_after_receive=[dict(receipt=e['receipt'],
                send_after_receive_ms=(e['receipt']['sent_ns']-end)/1e6,
                send_before_processing_ms=(processed['processing_ns']-e['receipt']['sent_ns'])/1e6,
                pre_action_received_ns=e['pre_action_observation']['received_ns']) for e in commands]))
    report=dict(groups=result,event_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        note='Original X5 receive timestamps remain the start of stop statistics. No command is omitted because processing began later.')
    a.output.write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
