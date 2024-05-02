import time


from rclpy.executors import SingleThreadedExecutor
import ros2_asyncio

from demo_setup import demos


@demos
def multiple_tasks(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)
    should_run = True

    async def _do_task(name: str, sleep_sec: int):
        nonlocal should_run
        while should_run:
            await ros2_asyncio.sleep(simple_node, sleep_sec)
            print(f'{time.perf_counter():.2f}: Hello from {name}')

    async def sentinal_task(sleep_sec: int):
        nonlocal should_run
        await ros2_asyncio.sleep(simple_node, sleep_sec)
        should_run = False

    executor.create_task(_do_task("task1", 1))
    executor.create_task(_do_task("task2", 2))
    executor.create_task(_do_task("task3", 3))
    f = executor.create_task(sentinal_task(10))
    executor.spin_until_future_complete(f)


if __name__ == "__main__":
    multiple_tasks()
