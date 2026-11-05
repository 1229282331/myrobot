"""Duplex hardware control bridge, with bounded outstanding acknowledgements.

Input is forwarded when it arrives, independently of the return direction. No
input queue, retransmission or catch-up loop is maintained in this bridge.
"""
import asyncio
import json
import time


class StreamingGatewayConnection:
    def __init__(self,ws,hello,notify):
        self.ws=ws;self.status=hello['status'];self.notify=notify
        self.send_lock=asyncio.Lock();self.requests={};self.next_id=0
        self.reader=asyncio.create_task(self.receive())

    async def send(self,data,future=None,notify=True):
        async with self.send_lock:
            if self.reader.done():
                await self.reader
                raise ConnectionError('Control response channel ended')
            if len(self.requests)>=32:raise ConnectionError('Too many unacknowledged control inputs; no buffering or replay')
            self.next_id+=1;identity=self.next_id;now=time.perf_counter_ns()
            payload=dict(data,session=self.status['session'],request_id=identity,pc_received_monotonic_ns=now)
            if data.get('operation')=='input':payload['pc_input_monotonic_ns']=now
            self.requests[identity]=dict(sent_ns=now,future=future,notify=notify,client_id=data.get('request_id'))
            try:await asyncio.wait_for(self.ws.send(json.dumps(payload,allow_nan=False)),.05)
            except BaseException:
                self.requests.pop(identity,None)
                raise

    async def rpc(self,data):
        future=asyncio.get_running_loop().create_future()
        try:
            await self.send(data,future=future,notify=False)
            return await asyncio.wait_for(future,1)
        finally:
            # A disconnect can fail the reader and this future before send
            # returns. Consume that exception even when no waiter was reached.
            if not future.done():future.cancel()
            elif not future.cancelled():future.exception()

    async def receive(self):
        error=None
        try:
            async for raw in self.ws:
                reply=json.loads(raw);identity=reply.get('request_id')
                if identity not in self.requests:raise ConnectionError('Uncorrelated control reply')
                request=self.requests.pop(identity)
                if 'status' in reply:self.status=reply['status']
                if reply.get('rpc_timing'):
                    reply['rpc_timing'].update(pc_gateway_sent_ns=request['sent_ns'],pc_gateway_reply_ns=time.perf_counter_ns())
                future=request['future']
                if future is not None and not future.done():future.set_result(reply)
                if request['notify']:
                    reply['request_id']=request['client_id']
                    await asyncio.wait_for(self.notify(reply),.1)
        except asyncio.CancelledError:raise
        except Exception as exc:error=exc;raise
        finally:
            for request in self.requests.values():
                future=request['future']
                if future is not None and not future.done():future.set_exception(error or ConnectionError('Control connection closed'))

    async def close(self):
        self.reader.cancel()
        await asyncio.gather(self.reader,return_exceptions=True)
        await self.ws.close()
