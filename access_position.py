import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from rclpy.qos import ReliabilityPolicy, QoSProfile

class position(Node):
    def __init__(self):
        super().__init__("position")
        self.pos_position = self.create_subscription(TFMessage, "tf", self.call_back, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        print("Hey Dude")

    def call_back(self,msg):
        # self.get_logger().info('I heard: "%s"' % msg.transforms[0].header)
        # self.get_logger().info('I heard: "%s"' % msg.transforms[0].transform.rotation)
        # self.get_logger().info('I heard: "%s"' % msg.transforms[0].transform.rotation.x)
        self.get_logger().info('I heard: "%s"' % msg.transforms[0].transform.translation)
        self.get_logger().info('I heard: "%s"' % msg.transforms[0].transform.translation.x)

def main():

    rclpy.init()
    listenNode = position()
    rclpy.spin(listenNode)
    
if __name__ == '__main__':
    main()