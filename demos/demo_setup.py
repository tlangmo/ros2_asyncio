
import rclpy
import functools


def demos(func):
    """Decorator to setup and teardown rclpy for demos."""
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        # Setup code: initialize context and node
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            'TestExecutor', context=context)

        try:
            # Execute the decorated function, passing the node
            result = func(node, *args, **kwargs)
        finally:
            # Teardown code: destroy the node and shutdown rclpy
            node.destroy_node()
            rclpy.shutdown(context=context)

        return result
    return wrapper
