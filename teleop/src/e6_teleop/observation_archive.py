"""Reliable native media retrieval on a paced, independent loopback SSH path."""
import asyncio
import hashlib
import json
import zstandard
import websockets


async def begin_archive(gateway, instance, session):
    async with websockets.connect(gateway+'/ws/archive',proxy=None,compression=None,max_size=65536) as ws:
        await ws.send(json.dumps(dict(operation='begin',instance=instance,session=session)))
        reply=json.loads(await asyncio.wait_for(ws.recv(),10))
        if not reply.get('ok'):raise ValueError(reply.get('error','Archive begin rejected'))


async def retrieve_archive(gateway, instance, recorder):
    session=recorder.directory.name
    async with websockets.connect(gateway+'/ws/archive',proxy=None,compression=None,max_size=65536,max_queue=2) as ws:
        async def request(operation,**kwargs):
            await ws.send(json.dumps(dict(operation=operation,instance=instance,session=session,**kwargs)))
            reply=json.loads(await asyncio.wait_for(ws.recv(),30))
            if not reply.get('ok'):raise ValueError(reply.get('error','Archive operation rejected'))
            return reply
        reply=await request('seal')
        if reply['archive'].get('error'):raise ValueError(reply['archive']['error'])
        for spec in recorder.specs.values():
            reply=await request('fetch',source=spec['source'],id=spec['id'])
            count=reply['wire_bytes']
            if not 0<count<=2*1024*1024:raise ValueError('Invalid native archive wire size')
            chunks=[];size=0
            while size<count:
                chunk=await asyncio.wait_for(ws.recv(),10)
                if not isinstance(chunk,bytes) or size+len(chunk)>count:raise ValueError('Invalid archive media block')
                chunks.append(chunk);size+=len(chunk)
            payload=b''.join(chunks)
            if hashlib.sha256(payload).hexdigest()!=reply['wire_sha256']:raise ValueError('Archive wire checksum mismatch')
            data=zstandard.ZstdDecompressor().decompress(payload,max_output_size=2*1024*1024)
            # Cancellation may not interrupt a thread halfway through a write.
            # Finish that owned write before closing the episode or allowing another.
            write=asyncio.create_task(asyncio.to_thread(recorder.import_media,spec,data,reply['entry']))
            try:await asyncio.shield(write)
            except asyncio.CancelledError:
                await write
                raise
        await request('release')
