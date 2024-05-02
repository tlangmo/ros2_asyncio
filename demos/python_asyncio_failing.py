import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import asyncio


class SleepABitService(Node):
    def __init__(self):
        super().__init__('sleep_service')
        self.service = self.create_service(
            Trigger, '/dummy/sleep5', self.sleep10_callback, callback_group=ReentrantCallbackGroup())

    async def sleep10_callback(self, request, response):
        self.get_logger().info('Sleeping for 5 seconds...')
        # That won't work without an asyncio eventloop.
        await asyncio.sleep(4)
        self.get_logger().info('done')
        response.success = True
        response.message = 'ok'
        return response


def main(args=None):
    rclpy.init(args=args)
    sleep_service = SleepABitService()
    executor = SingleThreadedExecutor()
    executor.add_node(sleep_service)
    try:
        print("Execute `ros2 service call /dummy/sleep5 std_srvs/srv/Trigger` in another terminal.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sleep_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
