"""Measure live wire latency independently of rendering/recording (read-only)."""
import argparse
import asyncio
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import socket
import sys
import time
import numpy as np
import websockets
from e6_teleop.tunnel import Tunnel
from e6_teleop.observation_clock import TransportClock
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'device_gateway'))
from e6_observer.protocol import decode_wire


def distribution(values):
    if len(values) == 0:
        return None
    return dict(zip(('min', 'median', 'p95', 'p99', 'max'),
                    map(float, np.percentile(values, [0, 50, 95, 99, 100]))))


async def measure(seconds):
    clock = TransportClock()
    samples = {'observations': [], 'media': [], 'probes': []}
    sent = {}
    async with websockets.connect('ws://127.0.0.1:8767/ws/observations',
                                  compression=None, proxy=None, max_queue=2) as telemetry:
        async def probe():
            while True:
                now = time.perf_counter_ns()
                sent[str(now)] = now
                await telemetry.send(json.dumps(dict(operation='clock_probe', nonce=str(now))))
                await asyncio.sleep(.5)

        async def stream(ws, name):
            cache = {}
            async for message in ws:
                arrived = time.perf_counter_ns()
                frame = decode_wire(message, cache)
                decoded = time.perf_counter_ns()
                if 'clock_probe' in frame:
                    start = sent.pop(frame['clock_probe'])
                    accepted = clock.update(start, arrived, frame['x5_monotonic_ns'])
                    samples['probes'].append(dict(rtt_ms=(arrived-start)/1e6, accepted=accepted))
                    continue
                if frame.get('error'):
                    samples[name].append(dict(error=frame['error']))
                    continue
                samples[name].append(dict(
                    pc_received_ns=arrived, decode_ms=(decoded-arrived)/1e6,
                    wire_bytes=len(message), frame=frame['observation']['frame_id'],
                    transport_upper_ms=clock.age_upper_ms(frame['observation']['gateway_monotonic_ns'], arrived),
                    source_age_ms={n:frame['sources'][n]['age_ms'] for n in ('robot','rgb','depth')},
                    stats={k:frame['stats'].get(k) for k in ('publisher','media_publisher','rgb_processing_ms','depth_processing_ms','decode_pending_max')},
                    valid=frame['observation']['valid']))
        tasks = [asyncio.create_task(probe()), asyncio.create_task(stream(telemetry, 'observations'))]
        await asyncio.sleep(2)
        try:
            async with websockets.connect('ws://127.0.0.1:8767/ws/media',
                                          compression=None, proxy=None, max_size=8*1024*1024,
                                          max_queue=2) as media:
                tasks.append(asyncio.create_task(stream(media, 'media')))
                await asyncio.sleep(seconds)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
    return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--wifi-low-latency', action='store_true')
    parser.add_argument('--nodelay', action='store_true')
    parser.add_argument('--allow-nagle', action='store_true',help='Diagnostic only: permit SSH TCP small-packet coalescing')
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    tunnel = Tunnel('192.168.1.13', remote_root='/root/e6-observer-20261004',
                    wifi_low_latency=args.wifi_low_latency)
    try:
        tunnel.start()
        sock = tunnel.client.get_transport().sock
        before = sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
        if args.nodelay:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if args.allow_nagle:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
        _, ping, ping_error = tunnel.client.exec_command(
            f'ping -n -i 0.1 -c {int(args.seconds*10)} 192.168.1.7')
        samples = asyncio.run(measure(args.seconds))
        result = dict(measured_utc=datetime.now(timezone.utc).isoformat(),
                      seconds=args.seconds, wifi_low_latency=args.wifi_low_latency,
                      ssh_tcp_nodelay_before=before,
                      ssh_tcp_nodelay=sock.getsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY),
                      ping_under_media=ping.read().decode(), ping_error=ping_error.read().decode(),
                      samples=samples)
        metrics = {}
        for name in ('observations', 'media'):
            rows = [s for s in samples[name] if 'error' not in s]
            times = [s['pc_received_ns'] for s in rows]
            metrics[name] = dict(samples=len(rows),
                                wire_MBps=sum(s['wire_bytes'] for s in rows)/args.seconds/1e6,
                                interval_ms=distribution(np.diff(times)/1e6),
                                transport_upper_ms=distribution([s['transport_upper_ms'] for s in rows if s['transport_upper_ms'] is not None]),
                                decode_ms=distribution([s['decode_ms'] for s in rows]))
        metrics['probe_rtt_ms'] = distribution([p['rtt_ms'] for p in samples['probes']])
        result['metrics'] = metrics
        print(json.dumps(metrics, ensure_ascii=False), flush=True)
        (args.evidence/'wire_latency.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    finally:
        result = tunnel.close()
        (args.evidence/'gateway_process_log.txt').write_text(''.join(getattr(tunnel, 'logs', [])), encoding='utf-8')
        (args.evidence/'resource_exit.json').write_text(json.dumps(result, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
