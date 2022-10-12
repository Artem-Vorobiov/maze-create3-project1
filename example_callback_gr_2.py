import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty

import access_position as pd # NEW


class CallbackGroupDemo(Node):
    def __init__(self):
        super().__init__('client_node')

        # client_cb_group = None
        # timer_cb_group = None
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.client = self.create_client(Empty, 'test_service', callback_group=client_cb_group)
        self.call_timer = self.create_timer(2, self._timer_cb, callback_group=timer_cb_group)

    def _timer_cb(self):
        self.get_logger().info('Sending request')
        _ = self.client.call(Empty.Request())
        self.get_logger().info('Received response')


if __name__ == '__main__':
    rclpy.init()
    node = CallbackGroupDemo()
    node_2 = pd.Position() # NEW

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node_2) # NEW


    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()