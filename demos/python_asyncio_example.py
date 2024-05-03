import asyncio
import time
""" Demonstration of Python's own asyncio library.
"""


async def my_coroutine_fast():
    for i in range(5):
        print(f"{time.time()}\t my_coroutine_fast {i}")
        await asyncio.sleep(1)


async def my_coroutine_slow():
    for i in range(5):
        print(f"{time.time()}\t my_coroutine_slow {i}")
        await asyncio.sleep(3)


# Create a new event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

try:
    loop.run_until_complete(asyncio.gather(
        my_coroutine_fast(), my_coroutine_slow()))
finally:
    # Close the loop to clean up
    loop.close()
