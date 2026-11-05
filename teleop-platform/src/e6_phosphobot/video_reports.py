"""Bounded local diagnostics storage, independent of the X5/control session."""
import asyncio
import hashlib
import json
from pathlib import Path
import re
import time
import uuid

from fastapi import HTTPException, Request
from fastapi.responses import FileResponse

MAX_REPORT_BYTES = 8 * 1024 * 1024


def save_report(directory, body):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    name = 'video-' + uuid.uuid4().hex + '.json'
    target = directory / name
    pending = directory / (name + '.part')
    try:
        with pending.open('xb') as stream:
            stream.write(body)
        pending.replace(target)
    except OSError:
        pending.unlink(missing_ok=True)
        raise
    return dict(name=name, bytes=len(body), sha256=hashlib.sha256(body).hexdigest(),
                pc_saved_monotonic_ns=time.perf_counter_ns(),
                url='/e6/diagnostics/video/' + name)


def install_video_reports(router, directory):
    directory = Path(directory)

    @router.post('/e6/diagnostics/video')
    async def store(request: Request):
        origin=request.headers.get('origin')
        if origin and origin != f'{request.url.scheme}://{request.url.netloc}':
            raise HTTPException(403, 'Report must come from this workstation page')
        if request.headers.get('content-type', '').split(';')[0] != 'application/json':
            raise HTTPException(415, 'Expected a JSON video report')
        body = bytearray()
        async for chunk in request.stream():
            if len(body) + len(chunk) > MAX_REPORT_BYTES:
                raise HTTPException(413, 'Video report exceeds the 8 MiB limit')
            body.extend(chunk)
        try:
            def invalid_constant(value):
                raise ValueError('Non-finite value: ' + value)
            report = json.loads(body, parse_constant=invalid_constant)
            if (not isinstance(report, dict) or report.get('schema') != 2
                    or not isinstance(report.get('samples'), list)
                    or not isinstance(report.get('presentationWindows'), list)):
                raise ValueError('Unknown video report shape')
        except (ValueError, UnicodeDecodeError, RecursionError) as exc:
            raise HTTPException(422, 'Invalid video measurement report') from exc
        try:
            return await asyncio.to_thread(save_report, directory, bytes(body))
        except OSError as exc:
            raise HTTPException(507, 'Local diagnostic file could not be saved') from exc

    @router.get('/e6/diagnostics/video/{name}')
    def download(name: str):
        if not re.fullmatch(r'video-[0-9a-f]{32}\.json', name):
            raise HTTPException(404, 'Unknown video report')
        path = directory / name
        if not path.is_file():
            raise HTTPException(404, 'Unknown video report')
        return FileResponse(path, filename=name, media_type='application/json')
