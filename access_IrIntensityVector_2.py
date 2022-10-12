import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrIntensityVector

from rclpy.qos import ReliabilityPolicy, QoSProfile

class Sensors(Node):
    def __init__(self):
        super().__init__("FrontSensors")
        print("Inside the node")
        self.irSubscriber = self.create_subscription(IrIntensityVector, "ir_intensity", self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.ir_readings = []   # added
        

    def ir_callback(self,msg):
        print("Enter the call_back function")

        self.ir_readings = msg.readings     # added
        if self.ir_readings:
            for i in range(6):
                if self.ir_readings[i].value < 200:
                    print("The Reading are  < 200 !") 
                else:
                    print("The Readings are > 200")

def main():

    rclpy.init()
    listenNode = Sensors()
    rclpy.spin(listenNode)
    
if __name__ == '__main__':
    main()