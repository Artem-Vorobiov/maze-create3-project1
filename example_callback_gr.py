import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(Empty, 'test_service', callback=self.service_callback)

    def service_callback(self, request, result):
        self.get_logger().info('Received request, responding...')
        return result


if __name__ == '__main__':
    rclpy.init()
    
    node = ServiceNode()
    try:
        node.get_logger().info("Starting server node, shut down with CTRL-C")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()