import asyncio
import time
from phosphobot.endpoints import update


def test_version_fetch_does_not_stall_websocket_event_loop(monkeypatch):
    monkeypatch.setattr(update.platform,'system',lambda:'Windows')
    def slow_fetch(**kwargs):time.sleep(.35);return 'test'
    monkeypatch.setattr(update,'fetch_latest_brew_version',slow_fetch)
    async def run():
        timestamps=[]
        async def heartbeat():
            for _ in range(15):
                timestamps.append(time.perf_counter())
                await asyncio.sleep(.025)
        await asyncio.gather(heartbeat(),update.get_latest_available_version())
        return max(b-a for a,b in zip(timestamps,timestamps[1:]))
    assert asyncio.run(run())<.1
