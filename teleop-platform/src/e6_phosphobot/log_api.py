"""Read-only PC forwarding via the existing media SSH tunnel."""
import json
from urllib.parse import urlencode
from urllib.request import urlopen
from fastapi import HTTPException
from fastapi.responses import Response,StreamingResponse
from starlette.background import BackgroundTask
from .log_sources import SOURCES,CHUNK_BYTES


def install_log_api(router,media_url,local):
    def allowed(source):
        if source not in SOURCES:raise HTTPException(404,'Unknown log source')

    @router.get('/e6/logs')
    def sources():
        result=local.listing()
        try:
            with urlopen(media_url+'/logs',timeout=2) as response:remote=json.loads(response.read(16384))
            result['remote_session']=remote['session']
            result['sources']+= [row for row in remote['sources'] if row['id'] in ('control','camera','servo')]
        except (OSError,ValueError,KeyError):
            result['sources'] += [dict(id=name,label=name.title(),available=False) for name in ('control','camera','servo')]
        return result

    @router.get('/e6/logs/{source}/stream')
    def stream(source:str,cursor:str=''):
        allowed(source)
        if len(cursor)>1024:raise HTTPException(400,'Invalid log cursor')
        try:
            if source=='pc':data,headers=local.read(source,cursor)
            else:
                with urlopen(media_url+'/logs/'+source+'/stream?'+urlencode({'cursor':cursor}),timeout=2) as remote:
                    data=remote.read(CHUNK_BYTES+1)
                    if len(data)>CHUNK_BYTES:raise ValueError('Log response exceeds chunk budget')
                    headers={key:remote.headers[key] for key in ('X-Log-Cursor','X-Log-Reset','Cache-Control')}
            return Response(data,media_type='text/plain; charset=utf-8',headers=headers)
        except (OSError,ValueError,KeyError) as exc:raise HTTPException(503,'Log unavailable: '+str(exc))

    @router.get('/e6/logs/{source}/download')
    def download(source:str):
        allowed(source)
        try:
            if source=='pc':reader,size=local.download(source)
            else:
                reader=urlopen(media_url+'/logs/'+source+'/download',timeout=3)
                size=int(reader.headers['Content-Length'])
        except (OSError,ValueError,KeyError) as exc:raise HTTPException(503,'Log unavailable: '+str(exc))
        def chunks():
            try:
                remaining=size
                while remaining:
                    data=reader.read(min(65536,remaining))
                    if not data:break
                    remaining-=len(data);yield data
            finally:reader.close()
        return StreamingResponse(chunks(),media_type='text/plain; charset=utf-8',
            headers={'Content-Disposition':f'attachment; filename="{source}.log"','Content-Length':str(size),
                     'Cache-Control':'no-store'},background=BackgroundTask(reader.close))
