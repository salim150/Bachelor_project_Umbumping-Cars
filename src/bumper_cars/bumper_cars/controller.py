#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State
class Controller(Node):

    def __init__(self):
        super().__init__("bumper_car_controller")
        self.previous_x = 0
        self.get_logger().info("Controller has been started")

        self.control_publisher_ = self.create_publisher(ControlInputs, "/robot_control", 10)
        self.pose_subscriber_ = self.create_subscription(State,
                                                         "/robot_state", 
                                                         self.pose_callback, 10)
        
    def pose_callback(self, pose: State):
    
        cmd = ControlInputs()
        cmd.delta = 30.0
        cmd.throttle = 2.0
        self.control_publisher_.publish(cmd)



   

def main(args=None):
    rclpy.init(args=args)

    node = Controller()
    rclpy.spin(node)

    rclpy.shutdown()