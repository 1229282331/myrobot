"""Parent EOF maps to uvicorn's ordinary SIGINT path, after lifespan startup."""
import asyncio
from contextlib import asynccontextmanager
import os
import signal
import sys
import threading


def wait_for_parent(fd, notify):
    # Raw os.read avoids BufferedReader's interpreter-shutdown lock on Windows.
    while True:
        try:
            data = os.read(fd, 64)
        except OSError:
            data = b''
        if not data or b'stop' in data:
            notify()
            return


def install_parent_shutdown(app):
    original = app.router.lifespan_context
    @asynccontextmanager
    async def lifespan(application):
        async with original(application):
            loop = asyncio.get_running_loop()
            closing = threading.Event()
            def notify():
                if not closing.is_set():
                    loop.call_soon_threadsafe(signal.raise_signal, signal.SIGINT)
            threading.Thread(target=wait_for_parent, args=(sys.stdin.fileno(), notify),
                             name='launcher-parent', daemon=True).start()
            try:
                yield
            finally:
                closing.set()
    app.router.lifespan_context = lifespan
