from rclpy.task import Future
import time
from rclpy.node import Node
from rclpy.task import Future
import rclpy
from typing import Union
from collections.abc import Awaitable
import asyncio
import inspect


def ensure_future(node, coro_or_future: Union[Future, Awaitable]):
    """Wrap a coroutine or an awaitable in a Ros2 future.

    If the argument is a Future, it is returned directly.
    """
    if isinstance(coro_or_future, Future):
        return coro_or_future
    if not asyncio.iscoroutine(coro_or_future):
        if inspect.isawaitable(coro_or_future):
            async def _wrap_awaitable(awaitable):
                return await awaitable
            coro_or_future = _wrap_awaitable(coro_or_future)
            should_close = False
        else:
            raise TypeError('An rclpy.Future, a coroutine or an awaitable '
                            'is required')

    try:
        assert node and node.executor
        return node.executor.create_task(coro_or_future)
    except RuntimeError:
        raise


async def sleep(node: Node, timeout_sec: float):
    """Ros2 compatible coroutine sleep function.

    It works similar to asyncio.sleep() but it is compatible with the Ros2 event loop.
    @see https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b
    """
    my_future = Future()

    def _on_done():
        my_future.set_result(True)
    timer = node.create_timer(timeout_sec, callback=_on_done,
                              callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
    await my_future
    # Important! Clean up the timer properly
    node.destroy_timer(timer)


async def wait_for(node: Node, coro_or_future: Union[Future, Awaitable], timeout_sec: float):
    """Ros2 compatible coroutine to wait for a future with timeout.

    It works similar to asyncio.wait_for() but it is compatible with the Ros2 event loop.
    @see https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b
    """
    start_time = time.time()

    coro_or_future = ensure_future(node, coro_or_future)
    if not coro_or_future.done():
        wait_future = Future()

        def _on_finished(f=None):
            wait_future.set_result(True)
        # create a timer to check in timeout_sec if the future is done
        timer = node.create_timer(timeout_sec, callback=_on_finished,
                                  callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        try:
            # register a callback to stop early if the future finished before the timeout is reached
            coro_or_future.add_done_callback(_on_finished)
            # yield control to the Ros2 event loop.
            await wait_future
            if not coro_or_future.done() and (time.time() - start_time > timeout_sec):
                raise TimeoutError(
                    "Timeout while waiting for future to complete")
            if coro_or_future.exception() is not None:
                raise coro_or_future.exception()
        finally:
            node.destroy_timer(timer)
    return coro_or_future.result()
