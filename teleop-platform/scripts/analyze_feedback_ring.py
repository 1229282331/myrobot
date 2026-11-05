"""Offline correlation of a bounded PCAP ring with immutable SDK audit bytes."""
from collections import Counter, defaultdict
from pathlib import Path
import argparse
import base64
import bisect
import gzip
import hashlib
import json
import socket
import struct

import dpkt


def quant(values):
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), **{key: values[round((len(values)-1)*p)]
                for key, p in [('p50', .5), ('p95', .95), ('p99', .99), ('max', 1)]})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    parser.add_argument('audit', type=Path)
    args = parser.parse_args()
    base = args.capture
    clock = json.loads((base/'clock.json').read_text())
    packets = []
    files = []
    paths = [path for path in base.glob('wire.pcap*')
             if path.suffix == '.gz' or not Path(str(path)+'.gz').exists()]
    for path in sorted(paths):
        stamps = []
        opener = gzip.open if path.suffix == '.gz' else open
        with opener(path, 'rb') as source:
            for stamp, data in dpkt.pcap.Reader(source):
                ns = round(float(stamp)*1e9)
                packets.append((ns, data))
                stamps.append(ns)
        files.append(dict(name=path.name, bytes=path.stat().st_size,
                          packets=len(stamps), first_wall_ns=min(stamps) if stamps else None,
                          last_wall_ns=max(stamps) if stamps else None))
    # File suffixes are cyclic slots, not chronology.
    packets.sort(key=lambda item: item[0])
    streams = defaultdict(list)
    arrivals = []
    for stamp, packet in packets:
        ip = dpkt.ethernet.Ethernet(packet).data
        if not isinstance(ip, dpkt.ip.IP) or not isinstance(ip.data, dpkt.tcp.TCP):
            continue
        tcp = ip.data
        if socket.inet_ntoa(ip.src) != '192.168.1.4' or tcp.sport != 30004 or not tcp.data:
            continue
        streams[(socket.inet_ntoa(ip.dst), tcp.dport)].append((tcp.seq, bytes(tcp.data), stamp))
        arrivals.append(stamp)
    counts = Counter()
    wire = defaultdict(list)
    for peer, segments in streams.items():
        first = segments[0][0]
        segments = sorted((((seq-first+2**31) % 2**32)-2**31, data, stamp)
                          for seq, data, stamp in segments)
        runs = []
        start = None
        buf = bytearray()
        spans = []
        for seq, data, stamp in segments:
            if start is None or seq > start+len(buf):
                if start is not None:
                    runs.append((start, bytes(buf), spans))
                    counts['tcp_missing_bytes'] += seq-start-len(buf)
                start = seq
                buf = bytearray()
                spans = []
            offset = seq-start
            overlap = max(0, len(buf)-offset)
            if overlap:
                size = min(overlap, len(data))
                counts['overlap_bytes'] += size
                if bytes(buf[offset:offset+size]) != data[:size]:
                    counts['conflicting_segments'] += 1
            if len(data) > overlap:
                spans.append((len(buf), len(buf)+len(data)-overlap, stamp))
                buf.extend(data[overlap:])
        if start is not None:
            runs.append((start, bytes(buf), spans))
        for start, data, spans in runs:
            offsets = [span[0] for span in spans]
            index = 0
            while index+1440 <= len(data):
                if (struct.unpack_from('<Q', data, index)[0] != 1440 or
                        struct.unpack_from('<Q', data, index+48)[0] != 0x0123456789abcdef):
                    counts['unframed_bytes'] += 1
                    index += 1
                    continue
                part = max(0, bisect.bisect_right(offsets, index)-1)
                times = []
                while part < len(spans) and spans[part][0] < index+1440:
                    if spans[part][1] > index:
                        times.append(spans[part][2])
                    part += 1
                digest = hashlib.sha256(data[index:index+1440]).hexdigest()
                wire[digest].append(dict(wall_ns=max(times), peer=peer, tcp_seq=start+index))
                counts['wire_frames'] += 1
                index += 1440
            counts['trailing_bytes'] += len(data)-index
    sample_path = base/'samples.jsonl'
    opener = open if sample_path.exists() else gzip.open
    if not sample_path.exists():
        sample_path = base/'samples.jsonl.gz'
    with opener(sample_path, 'rt', encoding='utf8') as source:
        samples = [json.loads(line) for line in source]
    brackets = [clock['before'], clock['after']]+[s['clock'] for s in samples if 'clock' in s]
    lo = min(p['mono_before_ns']-p['wall_ns'] for p in brackets)-1000
    hi = max(p['mono_after_ns']-p['wall_ns'] for p in brackets)+1000
    audit_hits = defaultdict(list)
    with gzip.open(args.audit/'raw.jsonl.gz', 'rt', encoding='utf8') as source:
        for line in source:
            row = json.loads(line)
            digest = hashlib.sha256(base64.b64decode(row['raw_b64'])).hexdigest()
            if digest in wire:
                audit_hits[digest].append(dict(sequence=row['sequence'], received_ns=row['received_ns']))
    matches = []
    for digest, rows in audit_hits.items():
        if len(rows) != 1 or len(wire[digest]) != 1:
            continue
        row = rows[0]
        wall = wire[digest][0]['wall_ns']
        matches.append(dict(**row, wire_wall_ns=wall,
                            lower_ms=(row['received_ns']-wall-hi)/1e6,
                            upper_ms=(row['received_ns']-wall-lo)/1e6))
    phases = []
    slow = []
    with gzip.open(args.audit/'events.jsonl.gz', 'rt', encoding='utf8') as source:
        for line in source:
            event = json.loads(line)
            if event['event'] == 'cycle':
                phase = event['state']['recording']['phase']
                if not phases or phases[-1]['phase'] != phase:
                    phases.append(dict(at_ns=event['at_ns'], phase=phase))
            if event['event'] == 'feedback_transport' and event['data']['event'] == 'receive_timing':
                slow.append(event['data'])
    windows = []
    for event in slow:
        begin = event.get('previous_publish_end_ns') or event['recv_started_ns']
        end = event['publish_finished_ns']
        covered = min(arrivals)+hi <= begin and max(arrivals)+lo >= end
        phases_before = [p['phase'] for p in phases if p['at_ns'] <= begin]
        window_samples = [s for s in samples if begin-100_000_000 <= s['started_ns'] <= end+100_000_000]
        window_matches = [m for m in matches if begin <= m['received_ns'] <= end]
        during_arrivals = [v for v in arrivals if begin <= v+lo and v+hi <= end]
        windows.append(dict(event=event, fully_covered=covered,
                            phase=phases_before[-1] if phases_before else None,
                            recv_wait_ms=(event['recv_returned_ns']-event['recv_started_ns'])/1e6,
                            publish_wait_ms=(end-event['recv_returned_ns'])/1e6,
                            arrivals_during=len(during_arrivals),
                            wire_gap_ms=quant([(b-a)/1e6 for a, b in zip(during_arrivals, during_arrivals[1:])]),
                            receive_delay_upper_ms=quant([m['upper_ms'] for m in window_matches]),
                            samples=window_samples))
    fault_deltas = []
    for previous, current in zip(samples, samples[1:]):
        for role in ('gateway', 'receiver', 'camera'):
            a, b = previous.get(role), current.get(role)
            if role == 'receiver':
                a = a.get('stat') if a else None
                b = b.get('stat') if b else None
            if a and b and a['starttime'] == b['starttime'] and b['majflt'] > a['majflt']:
                fault_deltas.append(dict(role=role, begin_ns=previous['started_ns'],
                                         end_ns=current['finished_ns'], major=b['majflt']-a['majflt']))
    result = dict(method='Exact SHA256 SDK frame correlation; ring slots merged by packet time, TCP reassembled. Repeated payloads on either side excluded. Same X5 clock brackets plus 1us quantization; no cross-host subtraction.',
        files=files, counts=dict(counts), matched_frames=len(matches),
        ambiguous_wire_hashes=sum(len(v)>1 for v in wire.values()),
        ambiguous_audit_hashes=sum(len(v)>1 for v in audit_hits.values()),
        clock_interval_width_ms=(hi-lo)/1e6, phases=phases, fault_deltas=fault_deltas,
        retained_first_mono_interval=[min(arrivals)+lo, min(arrivals)+hi],
        retained_last_mono_interval=[max(arrivals)+lo, max(arrivals)+hi],
        receive_delay_upper_ms=quant([m['upper_ms'] for m in matches]),
        kernel_arrival_gap_ms=quant([(b-a)/1e6 for a, b in zip(arrivals, arrivals[1:])]),
        sampler_duration_ms=quant([(s['finished_ns']-s['started_ns'])/1e6 for s in samples]),
        sampler_interval_ms=quant([(b['started_ns']-a['started_ns'])/1e6 for a, b in zip(samples, samples[1:])]),
        slow_windows=[{k: v for k, v in w.items() if k != 'samples'} for w in windows],
        limitations=['Bounded ring may overwrite early capture; only retained coverage is claimed.',
                    'Periodic proc samples are observations, not scheduler traces or lock ownership; cadence is recorded in capture identity.',
                    'Capture and sampling add measured workload; no claim of zero perturbation.',
                    'Clock bounds assume no unobserved wall-time jump between bracket samples.'])
    (base/'correlation.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    with gzip.open(base/'slow-windows.json.gz', 'wt', encoding='utf8') as dest:
        json.dump(windows, dest)
    print(json.dumps({k: v for k, v in result.items() if k not in ('slow_windows', 'fault_deltas')}, indent=2))
    print('slow_windows', len(windows), 'fully_covered', sum(w['fully_covered'] for w in windows))


if __name__ == '__main__':
    main()
