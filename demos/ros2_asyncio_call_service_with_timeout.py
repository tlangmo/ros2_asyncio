from rclpy.node import Node
from time import perf_counter
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_services_default
import random
from rclpy.executors import SingleThreadedExecutor
import ros2_asyncio

from demo_setup import demos


class LuckyNumberNode(Node):
    """Node which publishes significant data."""

    def __init__(self, context):
        super().__init__('ServicesNode', context=context)
        self.initialize_services()

    def initialize_services(self):
        reentrant_service_callback_group = ReentrantCallbackGroup()
        self.create_service(Trigger, "service_node/lucky_number",
                            callback=self.run_trigger_callback,
                            qos_profile=qos_profile_services_default,
                            callback_group=reentrant_service_callback_group)
        self.get_logger().info("Created ROS2 services...")

    async def run_trigger_callback(self, request, response):
        self.get_logger().info(f'Received lucky number request. Waiting now for 5 seconds...')
        now = perf_counter()
        await ros2_asyncio.sleep(self, 5)
        response.success = True
        response.message = f'Processed callback in {perf_counter()-now:.2f} seconds. And here is your lucky number: {random.randint(1, 100)}'
        return response


@demos
def call_service(client_node):
    lucky_node = LuckyNumberNode(context=client_node.context)
    executor = SingleThreadedExecutor(context=client_node.context)
    executor.add_node(client_node)
    executor.add_node(lucky_node)

    async def call_service():
        client = client_node.create_client(
            Trigger, "service_node/lucky_number")
        request = Trigger.Request()
        try:
            now = perf_counter()
            client_node.get_logger().info(f'Requesting my lucky number...')
            result = await ros2_asyncio.wait_for(client_node, client.call_async(request), timeout_sec=2)
        except TimeoutError:
            client_node.get_logger().info(
                f'Service call timed out. Waited for {perf_counter()-now:.2f} seconds.')

    f = executor.create_task(call_service())
    executor.spin_until_future_complete(f)


if __name__ == "__main__":
    call_service()
