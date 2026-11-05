"""Opening races use the production admission gate and executor, no devices."""
import asyncio
import time

from e6_phosphobot.gateway import controller_admission
from test_gateway import setup


def test_replacement_waits_for_old_cleanup_and_starts_unprepared():
    async def run():
        executor, device, send, step, _ = setup('shadow')
        send(deadman=True, joint_rad_s=[0, 0, 0, 0, 0, .01]); step()
        old_session = executor.session
        assert old_session.ready and executor.candidate is not None
        slot = asyncio.Lock()
        await slot.acquire()
        entered = asyncio.Event()

        async def replacement():
            async with controller_admission(slot) as admitted:
                assert admitted and executor.claim('replacement')
                entered.set()
                assert executor.session is not old_session
                assert not executor.session.ready and executor.target is None
                step(3)
                assert not device.calls
                executor.release('replacement')

        waiting = asyncio.create_task(replacement())
        await asyncio.sleep(.01)
        assert not entered.is_set() and executor.owner == 'page'
        # The real handler releases its slot only after executor/episode cleanup.
        executor.release('page')
        slot.release()
        await waiting
        assert entered.is_set() and executor.owner is None and not slot.locked()
    asyncio.run(run())


def test_second_controller_timeout_cannot_stop_or_steal_active_session():
    async def run():
        executor, device, _, _, _ = setup('shadow')
        session = executor.session
        slot = asyncio.Lock(); await slot.acquire()
        began = asyncio.get_running_loop().time()
        async with controller_admission(slot, timeout_s=.025) as admitted:
            assert not admitted
        # CPython 3.10/Windows uses GetTickCount64 (15.625 ms resolution),
        # and asyncio may run a timer within one clock tick of its deadline.
        # Admission/ownership assertions below remain exact.
        resolution=time.get_clock_info('monotonic').resolution
        assert max(0., .025-resolution) <= asyncio.get_running_loop().time() - began < .5
        assert executor.owner == 'page' and executor.session is session and session.ready
        assert not device.calls and slot.locked()
        slot.release()
    asyncio.run(run())


def test_cancelled_waiter_does_not_unlock_existing_controller():
    async def run():
        slot = asyncio.Lock(); await slot.acquire()
        async def waiting():
            async with controller_admission(slot):
                raise AssertionError('Cancelled connection must not enter')
        task = asyncio.create_task(waiting())
        await asyncio.sleep(.01)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        assert slot.locked()
        slot.release()
        async with controller_admission(slot) as admitted:
            assert admitted
        assert not slot.locked()
    asyncio.run(run())


def test_failed_session_releases_admission_only_after_its_cleanup():
    async def run():
        slot = asyncio.Lock()
        finishing = asyncio.Event(); cleaned = asyncio.Event(); successor = asyncio.Event()
        async def first():
            async with controller_admission(slot) as admitted:
                assert admitted
                try:
                    raise RuntimeError('transport failed')
                finally:
                    finishing.set()
                    await cleaned.wait()
        async def second():
            async with controller_admission(slot) as admitted:
                assert admitted and cleaned.is_set()
                successor.set()
        task = asyncio.create_task(first()); await finishing.wait()
        next_task = asyncio.create_task(second()); await asyncio.sleep(.01)
        assert not successor.is_set()
        cleaned.set()
        try:
            await task
        except RuntimeError as exc:
            assert str(exc) == 'transport failed'
        await next_task
        assert successor.is_set() and not slot.locked()
    asyncio.run(run())
