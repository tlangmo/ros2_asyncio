import time


class Sleep:
    """sleep implementated as an 'awaitable' object."""

    def __init__(self, timeout):
        self.start = time.time()
        self.timeout = timeout

    def __await__(self):
        """While a generator itself, implementing `__await__` makes it
        awaitable and thus usable in an `await` expression."""
        while time.time() - self.start < self.timeout:
            yield


async def sleep(timeout):
    """sleep implemented as a coroutine."""
    start = time.time()

    class _Awaitable():
        def __await__(self):
            while time.time() - start < timeout:
                yield
    await _Awaitable()


class Task:
    """Wrapper for a coroutine.

    We need to use send() instead of next(), because coroutines are not
    generators!
    """

    def __init__(self, coro):
        self.coro = coro

    def call(self):
        try:
            # When we call send on a coroutine,
            # it resumes its execution from where it last left off
            # (at the last yield).
            self.coro.send(None)
        except StopIteration:
            # The coroutine is done
            return False
        return True


class EventLoop:
    def __init__(self):
        self.tasks = []

    def create_task(self, coro):
        task = Task(coro)
        self.tasks.append(task)

    def run_until_complete(self):
        while self.tasks:
            task = self.tasks.pop(0)
            if task.call():
                # not done yet, schedule again
                self.tasks.append(task)


async def my_coroutine_fast():
    for i in range(5):
        print(f"{time.time()}\t my_coroutine_fast {i}")
        await sleep(1)


async def my_coroutine_slow():
    for i in range(5):
        print(f"{time.time()}\t my_coroutine_slow {i}")
        await Sleep(3)

# Creating and running the event loop
loop = EventLoop()
loop.create_task(my_coroutine_fast())
loop.create_task(my_coroutine_slow())
loop.run_until_complete()
