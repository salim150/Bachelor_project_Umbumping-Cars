#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State
import message_filters

debug = False

class Controller(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.previous_x = 0

        self.control1_publisher_ = self.create_publisher(ControlInputs, "/robot1_control", 20)
        self.control2_publisher_ = self.create_publisher(ControlInputs, "/robot2_control", 20)
        
        state1_subscriber = message_filters.Subscriber(self, State, "/robot1_measurement")
        state2_subscriber = message_filters.Subscriber(self, State, "/robot2_measurement")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber], 2, 0.1, allow_headerless=True)
        ts.registerCallback(self.general_pose_callback)

        self.control1_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))
        self.control2_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))

        self.get_logger().info("Controller has been started")

    def general_pose_callback(self, state1: State, state2: State):
        
        self.pose1_callback(state1)
        self.pose2_callback(state2)
        
        
    def pose1_callback(self, pose: State):
    
        cmd = ControlInputs()
        cmd.delta =15.0
        cmd.throttle = 3.0
        self.control1_publisher_.publish(cmd)
        
        if debug:
            self.get_logger().info("Control input robot1, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))

    def pose2_callback(self, pose: State):
    
        cmd = ControlInputs()
        cmd.delta = 15.0
        cmd.throttle = 3.0
        self.control2_publisher_.publish(cmd)
        
        if debug:
            self.get_logger().info("Control input robot2, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))

   

def main(args=None):
    rclpy.init(args=args)

    node = Controller()
    rclpy.spin(node)

    rclpy.shutdown()