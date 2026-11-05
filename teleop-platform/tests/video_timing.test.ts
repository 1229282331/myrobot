import test from 'node:test';
import assert from 'node:assert/strict';
import {clockBound,frameLatency} from '../frontend/videoTiming.ts';

test('asymmetric network delay stays inside an interval, not a one-way RTT guess',()=>{
  // X5 offset=1000 ms; outbound 30 ms, inbound 10 ms, server 2 ms.
  const b=clockBound({client_sent_ms:100,client_received_ms:142,
    x5_received_ns:1130e6,x5_sent_ns:1132e6})!;
  assert.equal(b.lower,990);assert.equal(b.upper,1030);assert.equal(b.rtt,40);
  const l=frameLatency('E6:23:1050',200,b)!;
  assert.ok(l.lowerMs<=150&&l.upperMs>=150);
  assert.equal(l.frameId,23);
  assert.equal(l.basis,'legacy_unspecified');
  assert.ok(Math.abs(l.driftAndQuantizationMs-1.1)<1e-9); // includes clock exchange
});
test('backwards, invalid, old, or unsynchronized timing is unavailable',()=>{
  assert.equal(clockBound({client_sent_ms:10,client_received_ms:9,x5_received_ns:0,x5_sent_ns:0}),null);
  assert.equal(clockBound({client_sent_ms:10,client_received_ms:11,x5_received_ns:0,x5_sent_ns:2e6}),null);
  assert.equal(frameLatency('E6:1:5',1,null),null);
  const b={lower:0,upper:1,at:0,rtt:1};
  assert.equal(frameLatency('E6:1:5',3001,b),null);
  assert.equal(frameLatency('E6:NaN:5',5,b),null);
  assert.equal(frameLatency('bad',5,b),null);
  assert.equal(frameLatency('E6:1:5',NaN,b),null);
  assert.equal(frameLatency('E6::5',5,b),null);
});
test('new markers separate kernel receipt, SDK return and restart epochs',()=>{
  const epoch='1234567890abcdef'.repeat(2);
  const b={lower:0,upper:1,at:0,rtt:1};
  const kernel=frameLatency(`E6K:${epoch}:23:1050`,1200,b)!;
  const sdk=frameLatency(`E6S:${epoch}:23:1050`,1200,b)!;
  assert.equal(kernel.basis,'uvc_kernel_receipt');assert.equal(sdk.basis,'sdk_wait_return');
  assert.equal(kernel.captureEpoch,epoch);assert.equal(kernel.sourceMs,1050);
  assert.deepEqual(kernel.clock,b);
  assert.equal(frameLatency('E6K:bad:23:1050',1200,b),null);
  assert.equal(frameLatency(`E6K:${epoch}:23:Infinity`,1200,b),null);
  assert.equal(frameLatency(`E6K:${epoch}:23:`,1200,b),null);
});
