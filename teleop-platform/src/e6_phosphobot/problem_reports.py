"""Local, bounded problem bundles. Never opens an X5 or device connection.

Use stdlib ZIP/SHA and the existing immutable session/episode records. Original
records stay untouched. Truncation, redaction and missing evidence are explicit.
"""
import asyncio
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import threading
import uuid
import zipfile

from fastapi import HTTPException, Request
from fastapi.responses import FileResponse

MAX_FILE_BYTES = 256 * 1024
MAX_TOTAL_BYTES = 4 * 1024 * 1024
SESSION = re.compile(r'\d{8}-\d{6}-[0-9a-f]{8}')
EPISODE = re.compile(r'\d{8}T\d{6}Z_[0-9a-f]{8}')
NAME = re.compile(r'problem-[0-9a-f]{32}\.zip')
CONFIG_KEYS = ('session', 'stage', 'motion_engine', 'moveit_api', 'motion_profile',
               'servo_period_s', 'servo_duration_s', 'servo_aheadtime', 'servo_gain',
               'test_workspace', 'native_smoothing', 'preview_codec', 'latency_marker',
               'source_manifest', 'runtime_manifest', 'cpp_runtime_manifest')
SECRET_KEY = re.compile(r'password|passwd|secret|token|authorization|credential|private.?key|api.?key', re.I)


def scrub_text(value):
    value = re.sub(r'-----BEGIN [^-]*PRIVATE KEY-----.*?(?:-----END [^-]*PRIVATE KEY-----|\Z)',
                   '[REDACTED PRIVATE KEY]', value, flags=re.S)
    value = re.sub(r'(?i)(https?://)[^\s/:]+:[^\s/@]+@', r'\1[REDACTED]@', value)
    value = re.sub(r'(?i)(bearer\s+)[A-Za-z0-9._~+/-]+', r'\1[REDACTED]', value)
    value = re.sub(r'''(?im)(["']?(?:password|passwd|secret|api_key|access_token|token|authorization)["']?\s*[:=]\s*)("[^"\r\n]*"|'[^'\r\n]*'|[^\s,;}]+)''',
                   r'\1"[REDACTED]"', value)
    value = re.sub(r'(?:gh[pousr]_[A-Za-z0-9]{30,}|AKIA[A-Z0-9]{16}|sk-proj-[A-Za-z0-9_-]{30,})',
                   '[REDACTED]', value)
    return value


def scrub(value):
    if isinstance(value, dict):
        return {k: '[REDACTED]' if SECRET_KEY.search(k) else scrub(v) for k, v in value.items()}
    if isinstance(value, list):
        return [scrub(v) for v in value]
    return scrub_text(value) if isinstance(value, str) else value


def encoded(value, *, indent=2):
    return json.dumps(value, ensure_ascii=False, indent=indent, allow_nan=False).encode('utf8')


def safe_file(root, path):
    return not path.is_symlink() and path.resolve().is_relative_to(root.resolve()) and path.is_file()


def latest_directories(root, directory, pattern):
    if not directory.is_dir() or not directory.resolve().is_relative_to(root.resolve()):
        return []
    return sorted((p for p in directory.iterdir() if pattern.fullmatch(p.name)
                   and not p.is_symlink() and p.is_dir()), key=lambda p: p.name, reverse=True)[:2]


def build_report(root, snapshot):
    root = Path(root).resolve()
    directory = root / 'runtime/diagnostics'
    if not directory.resolve().is_relative_to(root):
        raise ValueError('Diagnostic directory outside workstation')
    directory.mkdir(parents=True, exist_ok=True)
    name = 'problem-' + uuid.uuid4().hex + '.zip'
    pending, target = directory / (name + '.part'), directory / name
    entries, omissions = [], []
    contents = {}

    def add(name, data, **meta):
        if len(data) > MAX_FILE_BYTES or sum(map(len, contents.values())) + len(data) > MAX_TOTAL_BYTES:
            omissions.append(dict(file=name, reason='bundle_byte_budget'))
            return
        contents[name] = data
        entries.append(dict(file=name, bytes=len(data), sha256=hashlib.sha256(data).hexdigest(), **meta))

    def include(path, *, config=False, tail=False):
        name = path.relative_to(root).as_posix()
        if not safe_file(root, path):
            omissions.append(dict(file=name, reason='missing_or_outside_workspace'))
            return
        try:
            with path.open('rb') as stream:
                size = stream.seek(0, 2)
                if size > MAX_FILE_BYTES and not tail:
                    omissions.append(dict(file=name, reason='file_byte_budget', source_bytes=size))
                    return
                stream.seek(max(0, size-MAX_FILE_BYTES) if tail else 0)
                data = stream.read(MAX_FILE_BYTES)
            truncated = size > len(data)
            if truncated:
                # Never interpret a partial first line as a complete event.
                data = data.split(b'\n', 1)[-1] if b'\n' in data else b''
            text = data.decode('utf8', errors='replace')
            if config:
                parsed = json.loads(text)
                filtered = {k: parsed[k] for k in CONFIG_KEYS if k in parsed}
                result = encoded(scrub(filtered))
            elif path.suffix == '.json':
                result = encoded(scrub(json.loads(text)))
            elif path.suffix == '.jsonl':
                # Parse complete records before redaction. Unstructured tails
                # could start inside a credential (e.g. PEM); never copy them.
                records = []
                rejected = 0
                for line in text.splitlines():
                    if not line.strip():
                        continue
                    try:
                        records.append(encoded(scrub(json.loads(line)), indent=None))
                    except (ValueError, TypeError, RecursionError):
                        rejected += 1
                result = b'\n'.join(records)
                if rejected:
                    omissions.append(dict(file=name, reason='invalid_jsonl_records', count=rejected))
            else:
                omissions.append(dict(file=name, reason='unstructured_log_excluded'))
                return
            add(name, result, source_bytes=size, truncated=truncated,
                transformed=bool(config or result != data), source_kind='local_record')
        except (OSError, ValueError, TypeError, RecursionError):
            omissions.append(dict(file=name, reason='unreadable_or_incomplete'))

    add('current-state.json', encoded(scrub(snapshot)), source_kind='cached_observation',
        transformed=True, truncated=False)
    for session in latest_directories(root, root/'runtime/sessions', SESSION):
        include(session/'deployment.json', config=True)
        for file in ('source-sha256.json', 'startup-state.json', 'result.json'):
            include(session/file)
        include(session/'lifecycle.jsonl', tail=True)
        for file in ('local-8765.log', 'ssh.log'):
            omissions.append(dict(file=(session/file).relative_to(root).as_posix(),
                                  reason='unstructured_log_excluded'))
    for episode in latest_directories(root, root/'runtime/episodes', EPISODE):
        for file in ('metadata.json', 'summary.json', 'verification.json', 'pc_transfer.json'):
            include(episode/file)
        include(episode/'associations.jsonl', tail=True)
        include(episode/'audit/events.jsonl', tail=True)
    include(root/'upstream.lock.json')
    include(root/'upstream/phosphobot/phosphobot/resources/dist/e6-build.json')
    manifest = dict(schema='e6-problem-report-v1', created_at=datetime.now(timezone.utc).isoformat(),
        scope='Cached current state, latest two local sessions and episodes, fixed dependency/build hashes.',
        device_access=False, original_records_modified=False, raw_images_included=False,
        privacy='Fixed structured files only, credential fields redacted; console/SSH logs, keys and images excluded. Review before sharing.',
        limitation='A bounded diagnostic copy, not a complete acceptance record. Sources may be stale or partial.',
        clock_note='Each source retains its own process/host monotonic clock; no cross-host subtraction.',
        entries=entries, omissions=omissions)
    try:
        with zipfile.ZipFile(pending, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
            for entry, data in contents.items():
                archive.writestr(entry, data)
            archive.writestr('manifest.json', encoded(manifest))
        pending.replace(target)
    except Exception:
        pending.unlink(missing_ok=True)
        raise
    return dict(name=name, bytes=target.stat().st_size, sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                files=len(entries), omissions=len(omissions), url='/e6/diagnostics/problem/'+name)


def install_problem_reports(router, root, snapshot):
    root = Path(root)
    busy = threading.Lock()
    jobs = set()

    def finished(task):
        jobs.discard(task)
        if not task.cancelled():
            task.exception()  # Consume errors even when the HTTP caller left.

    @router.post('/e6/diagnostics/problem')
    async def create(request: Request):
        origin = request.headers.get('origin')
        if origin and origin != f'{request.url.scheme}://{request.url.netloc}':
            raise HTTPException(403, 'Problem report must come from this workstation page')
        if not busy.acquire(blocking=False):
            raise HTTPException(409, 'A problem report is already being saved')
        try:
            state = snapshot()  # Observer replaces this object, never mutates it.
        except Exception:
            busy.release()
            raise

        def worker():
            try:
                return build_report(root, state)
            finally:
                # A disconnected/cancelled HTTP request does not cancel file IO.
                # Only the worker may release the lock when it has really ended.
                busy.release()

        task = asyncio.create_task(asyncio.to_thread(worker))
        jobs.add(task)
        task.add_done_callback(finished)
        try:
            return await asyncio.shield(task)
        except (OSError, ValueError, TypeError) as exc:
            raise HTTPException(507, 'Problem report could not be saved locally') from exc

    @router.get('/e6/diagnostics/problem/{name}')
    def download(name: str):
        path = root/'runtime/diagnostics'/name
        if not NAME.fullmatch(name) or not safe_file(root, path):
            raise HTTPException(404, 'Unknown problem report')
        return FileResponse(path, filename=name, media_type='application/zip')
