# program description:
# the created program sends the robot in a straight line towards the goal coordinates
# if a wall is found with the bumper, the robot starts wall following on the left
# if the left infrared sensors no longer detect a wall, the robot turns a left arc
# then the robot continues navigating to the goal coordinates
# when the robot position is past Y coordinate of 1.3m, the wall following and arc change from left to right
# the robot stops moving when at the goal coordinates

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveArc, WallFollow, NavigateToPosition
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector
from tf2_msgs.msg import TFMessage
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String

class listener(Node):
	def __init__(self):
		super().__init__("listener")

		# subscribe to sensors
		self.hazzard					= self.create_subscription(HazardDetectionVector, "hazard_detection", self.hazard_call_back, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
		self.irSubscriber				= self.create_subscription(IrIntensityVector, "ir_intensity", self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
		self.tfSubscriber				= self.create_subscription(TFMessage, "tf", self.tf_call_back, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

		# create action clients for each action used
		self.action_client_wallfollow	= ActionClient(self, WallFollow, "wall_follow")
		self.action_client_navtopos		= ActionClient(self, NavigateToPosition, 'navigate_to_position')
		self.action_client_forward		= ActionClient(self, DriveDistance, 'drive_distance')
		self.action_client_arc			= ActionClient(self, DriveArc, 'drive_arc')

		# create arrays to store sensor data in
		self.ir_readings = []
		self.hazard_list = []
		self.tf_coords   = []

		# set initial state
		self.state = "Navigation"
		# start navigating to end goal
		self.nav_to()

		# this program keeps running based on a timer callback
		# here we define the time period and the timer
		timer_period = 2.0 # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)

	# data callback for hazard detection (bumper)
	def hazard_call_back(self,msg):
		self.hazard_list = msg.detections

	# data callback for infrared sensors
	def ir_callback(self,msg):
		self.ir_readings = msg.readings

	# data callback for tf
	def tf_call_back(self,msg):
		self.tf_coords   = msg.transforms[0].transform


	# a timer callback function is used to switch between created states based on sensor input and specific conditions
	def timer_callback(self):
		# checks if a valid reading exists
		# without this the ir sensor data is sometimes called before any data exists
		if self.ir_readings: 
			print("ITERATION")
			print("")

			print(self.tf_coords.translation.y)									# show currant Y coords
			if self.tf_coords.translation.y < 1.3:								# if y less than 1.3 follow the instructions
				if len(self.hazard_list) == 1 and self.state == "Navigation":	# if bump wall, change state to wall follow
					self.state = "WallFollow"							
					print(self.hazard_list[0].type)								# if bump then shows 1
					print(self.hazard_list[0].header.frame_id)					# if bump shows the side that is bumped
					print(str(self.ir_readings[0].value) + " Zero")
					print(str(self.ir_readings[1].value) + " First")			# prints values of ir sensors 0 and 1
					print(self.state)
					print("We are in condition 1")
					print("")
					self.send_goal_wallFollow()									# entering a function

				elif self.state == "Navigation":
					print("Navigating to Goal")
					print("We are in condition 0")

				elif (self.ir_readings[0].value > 50 or self.ir_readings[1].value > 75) and self.state == "WallFollow":
					# these conditions maintain the robot to follow the wall.
					print(str(self.ir_readings[0].value) + " Zero")
					print(str(self.ir_readings[1].value) + " First")
					print("We are in condition 2")
					print(self.state)
					print("")
					self.send_goal_wallFollow()

				elif self.state == "ARC":
					# if we match the condition, that means we did an arch and ready to drive ro the goals
					# we change the state and navigate to the goal.
					print(str(self.ir_readings[0].value) + " Zero")
					print(str(self.ir_readings[1].value) + " First")
					print("Navigate Second Order")
					print(self.state)
					print("")
					self.state = "Navigation"
					self.nav_to()

				elif self.ir_readings[0].value < 50 and self.ir_readings[1].value < 70 and self.state == "WallFollow":
					# if we match these conditions then we detected the corner and thus we change our goal and state.	
					self.state = "ARC"
					print("We are in condition 3")
					print(str(self.ir_readings[0].value) + " Zero")
					print(str(self.ir_readings[1].value) + " First")
					print(self.state)
					print("")
					self.send_goal_arc()


			else:
				if len(self.hazard_list) == 1 and self.state == "Navigation":	# if bump wall, change state to wall follow
					self.state = "WallFollow"							
					print(self.hazard_list[0].type)								# if bump then shows 1
					print(self.hazard_list[0].header.frame_id)					# if bump shows the side that is bumped
					print(str(self.ir_readings[0].value) + " Zero")
					print(str(self.ir_readings[1].value) + " First")			# prints values of ir sensors 0 and 1
					print(self.state)
					print("We are in condition 1")
					print("")
					self.send_goal_wallFollow_Right()

				elif self.state == "Navigation":
					print("Navigating to Goal")
					print("We are in condition 0")

				elif (self.ir_readings[6].value > 50 or self.ir_readings[5].value > 75) and self.state == "WallFollow":
					# these conditions maintain the robot to follow the wall.
					print(str(self.ir_readings[6].value) + " Zero")
					print(str(self.ir_readings[5].value) + " First")
					print("We are in condition 2")
					print(self.state)
					print("")
					self.send_goal_wallFollow_Right()

				elif self.state == "ARC":
					# if we match the condition, that means we did an arch and ready to drive ro the goals
					# we change the state and navigate to the goal.
					print(str(self.ir_readings[6].value) + " Zero")
					print(str(self.ir_readings[5].value) + " First")
					print("Navigate Second Order")
					print(self.state)
					print("")
					self.state = "Navigation"
					self.nav_to()

				elif self.ir_readings[6].value < 50 and self.ir_readings[5].value < 70 and self.state == "WallFollow":
					# if we match these conditions then we detected the corner and thus we change our goal and state.	
					self.state = "ARC"
					print("We are in condition 3")
					print(str(self.ir_readings[6].value) + " Zero")
					print(str(self.ir_readings[5].value) + " First")
					print(self.state)
					print("")
					self.send_goal_arcRIGHT()



	# The following functions represent different actions. Each action is connected to a state in the program logic

	# navigate to position function
	# sends robot in a straight line towards the given goal coordinates
	def nav_to(self):
		# print check 
		print("navigating to goal")
		print("")
		# create instance of NavigateToPosition class
		move = NavigateToPosition.Goal()
		# set desired position
		move.goal_pose.pose.position.x = 3.0
		move.goal_pose.pose.position.y = 2.7
		# set desired orientation
		move.goal_pose.pose.orientation.x = 0.0
		move.goal_pose.pose.orientation.y = 0.0
		move.goal_pose.pose.orientation.z = 0.0
		move.goal_pose.pose.orientation.w = 1.0
		# waits for server to be free
		self.action_client_navtopos.wait_for_server()
		# send command
		self.action_client_navtopos.send_goal_async(move)


	# wall follow function, following wall on the left
	def send_goal_wallFollow(self):
		# print check
		print("It is moving LEFT!")  
		print("") 
		# create instance of WallFollow class
		move = WallFollow.Goal()
		# set which side of wall to follow (1 = left, -1 = right)
		move.follow_side = 1
		# set time for wall follow command
		move.max_runtime.sec = 1
		# wait for server to be free
		self.action_client_wallfollow.wait_for_server()
		# send command
		self.action_client_wallfollow.send_goal_async(move)

	# wall follow function, following wall on the right
	def send_goal_wallFollow_Right(self):
		# print check
		print("It is moving RIGHT!")  
		print("") 
		# create instance of WallFollow class
		move = WallFollow.Goal()
		# set which side of wall to follow (1 = left, -1 = right)
		move.follow_side = -1
		# set time for wall follow command
		move.max_runtime.sec = 1
		# wait for server to be free
		self.action_client_wallfollow.wait_for_server()
		# send command
		self.action_client_wallfollow.send_goal_async(move)


	# drive arc function for turning left
	def send_goal_arc(self):
		# print check
		print("ARC")
		print("")
		# create instance of DriveArc class
		move = DriveArc.Goal()
		# set trabnskation direction, angle and radius of the arc
		move.translate_direction = 1
		move.angle = 2.0 #radians
		move.radius = 0.3 #meters
		# wait for server to be free
		self.action_client_arc.wait_for_server()
		# send command
		self.action_client_arc.send_goal_async(move)

	# drive arc function for turning right
	def send_goal_arcRIGHT(self):
		# print check
		print("ARC")
		print("")
		# create instance of DriveArc class
		move = DriveArc.Goal()
		# set trabnskation direction, angle and radius of the arc
		move.translate_direction = 1
		move.angle = -2.0 #radians
		move.radius = 0.3 #meters
		# wait for server to be free
		self.action_client_arc.wait_for_server()
		# send command
		self.action_client_arc.send_goal_async(move)


# runs the created class function
def main():

	rclpy.init()
	listenNode = listener()
	rclpy.spin(listenNode)

if __name__ == '__main__':
	main()