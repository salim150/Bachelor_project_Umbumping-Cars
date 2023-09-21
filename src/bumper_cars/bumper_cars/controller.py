#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State
class Controller(Node):

    def __init__(self, robot_name):
        super().__init__(robot_name + "_controller")
        self.previous_x = 0
        self.get_logger().info("Controller has been started")

        self.control_publisher_ = self.create_publisher(ControlInputs, "/" + robot_name + "_control", 10)
        self.pose_subscriber_ = self.create_subscription(State,
                                                         "/" + robot_name + "_measurement", 
                                                         self.pose_callback, 10)
        
    def pose_callback(self, pose: State):
    
        cmd = ControlInputs()
        cmd.delta = 30.0
        cmd.throttle = 2.0
        self.control_publisher_.publish(cmd)



   

def main(args=None):
    rclpy.init(args=args)

    node1 = Controller("robot1")
    node2 = Controller("robot2")
    rclpy.spin(node1)
    rclpy.spin(node2)

    rclpy.shutdown()