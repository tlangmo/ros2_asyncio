import time


class EventLoop:
    def __init__(self):
        self.tasks = []

    def create_task(self, coro):
        """Simply keep a list of coroutines to run."""
        self.tasks.append(coro)

    def run_until_complete(self):
        while self.tasks:
            coro = self.tasks.pop(0)
            try:
                # start/resume execution. Note the "next" keyword,
                # which is used in the context of generators
                next(coro)
                # schedule at the end
                self.tasks.append(coro)
            except StopIteration:
                # The task is done becuase 'next' raised the StopIteration.,
                # Do not schedule task again
                pass


def sleep(timeout):
    """A generator function that yields control when a timeout has not been
    reached.

    This is a typical pattern to 'delay' a function
    """
    start = time.time()
    while time.time() - start < timeout:
        # this yield makes a normal function a generator function
        yield


def my_generator_fast():
    for i in range(5):
        print(f"{time.time()} my_generator_fast {i}")
        # we are yielding from a generator
        yield from sleep(1)


def my_generator_slow():
    for i in range(5):
        print(f"{time.time()} my_generator_slow {i}")
        # we are yielding from a generator
        yield from sleep(2)


# Creating and running the event loop
loop = EventLoop()
loop.create_task(my_generator_fast())
loop.create_task(my_generator_slow())
loop.run_until_complete()
