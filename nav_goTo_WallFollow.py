import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, DriveArc, RotateAngle
from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String

from irobot_create_msgs.action import DockServo
# /dock [irobot_create_msgs/action/DockServo]

from irobot_create_msgs.action import Undock
# /undock [irobot_create_msgs/action/Undock]

from irobot_create_msgs.msg import Dock
# /dock [irobot_create_msgs/msg/Dock]

from irobot_create_msgs.action import WallFollow
# /wall_follow [irobot_create_msgs/action/WallFollow]

# FINALE - WORKS
class Sensors(Node):
	def __init__(self):
		super().__init__("FrontSensors")
		print("Inside the SENSORS")
		self.irSubscriber = self.create_subscription(IrIntensityVector, "ir_intensity", self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
		self.ir_readings = []	# added

	def ir_callback(self,msg):
		print("Enter the call_back for SENSOR")
		self.ir_readings = msg.readings 	# added
		if self.ir_readings:
			for i in range(6):
				if self.ir_readings[i].value > 300:
					print("IF")
					controller4 = LetsComeBack()
					rclpy.spin(controller4)	
class GoTo(Node):
	def __init__(self):
		super().__init__("GoController")
		print("We're in a GO mode")
		self.action_client_drive = ActionClient(self, DriveDistance, "drive_distance")
		self.send_goal_drive_toWall()
		controller5 = Sensors()
		rclpy.spin(controller5)

	def send_goal_drive_toWall(self):
		print("Going to the wall!")
		move = DriveDistance.Goal()
		move.distance = 6.0
		move.max_translation_speed = 1.0
		self.action_client_drive.wait_for_server()

		#Send the goal
		return self.action_client_drive.send_goal_async(move)

class LetsComeBack(Node):
	print("We're inside Follow")
	def __init__(self):
		super().__init__("Controller")
		print("Wall Follwoing")
		self.action_client_drive = ActionClient(self, WallFollow, "wall_follow")
		self.send_goal_driveBack()

	def send_goal_driveBack(self):
		print("It is moving!")   
		move = WallFollow.Goal()
		move.follow_side = 1
		move.max_runtime.sec = 15
		self.action_client_drive.wait_for_server()

		#Send the goal
		return self.action_client_drive.send_goal_async(move)

class position(Node):
	print("Inside Position Node")
	def __init__(self):
		super().__init__("position")
		controller3 = GoTo()
		rclpy.spin(controller3)

def main():
	rclpy.init()
	controller1 = position()
	rclpy.spin(controller1)

if __name__ == '__main__':
	main()