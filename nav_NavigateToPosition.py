import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
#from irobot_create_msgs.msg import IrIntensityVector#required messages for tasks
from irobot_create_msgs.action import NavigateToPosition
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import ReliabilityPolicy, QoSProfile #donno why yet

class NavToPos(Node):
    def __init__(self):
        super().__init__("NavToPos")

        self.action_client_navtopos = ActionClient(self, NavigateToPosition, 'navigate_to_position')
        print("sending Goal")
        self.go()

    def go(self):
        move = NavigateToPosition.Goal()

        move.goal_pose.pose.position.x = 0.5
        move.goal_pose.pose.position.y = 0.0
        move.goal_pose.pose.orientation.x = 0.0
        move.goal_pose.pose.orientation.y = 0.0
        move.goal_pose.pose.orientation.z = 0.0
        move.goal_pose.pose.orientation.w = 1.0

        self.action_client_navtopos.wait_for_server()
        self.action_client_navtopos.send_goal_async(move)


def main():
    rclpy.init()
    print("Starting")

    testrun = NavToPos()
    print("Spinning")
    rclpy.spin(testrun)
    

if __name__ == '__main__':
    main()
