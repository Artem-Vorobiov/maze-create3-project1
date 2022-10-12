import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector

from rclpy.qos import ReliabilityPolicy, QoSProfile

class listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.hazzard = self.create_subscription(HazardDetectionVector, "hazard_detection", self.call_back, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        print("Hey Dude")

    def call_back(self,msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        print(msg)

def main():

    rclpy.init()
    listenNode = listener()
    rclpy.spin(listenNode)
    
if __name__ == '__main__':
    main()