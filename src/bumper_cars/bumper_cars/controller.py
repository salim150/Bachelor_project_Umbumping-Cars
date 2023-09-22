#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State
import message_filters

class Controller(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.previous_x = 0

        self.control1_publisher_ = self.create_publisher(ControlInputs, "/robot1_control", 1)
        self.control2_publisher_ = self.create_publisher(ControlInputs, "/robot2_control", 1)

        """self.pose1_subscriber_ = self.create_subscription(State,
                                                         "/robot1_measurement", 
                                                         self.pose1_callback, 10)
        self.pose2_subscriber_ = self.create_subscription(State,
                                                         "/robot2_measurement", 
                                                         self.pose2_callback, 1)"""
        
        state1_subscriber = message_filters.Subscriber(self, State, "/robot1_state")
        state2_subscriber = message_filters.Subscriber(self, State, "/robot2_state")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber], 2, 0.1, allow_headerless=True)
        ts.registerCallback(self.general_pose_callback)


        self.get_logger().info("Controller has been started")

    def general_pose_callback(self, state1: State, state2: State):
        
        self.pose1_callback(state1)
        self.pose2_callback(state2)
        
        
    def pose1_callback(self, pose: State):
    
        cmd = ControlInputs()
        cmd.delta = 30.0
        cmd.throttle = 2.0
        self.control1_publisher_.publish(cmd)

    def pose2_callback(self, pose: State):
    
        cmd = ControlInputs()
        cmd.delta = 30.0
        cmd.throttle = 1.0
        self.control2_publisher_.publish(cmd)

   

def main(args=None):
    rclpy.init(args=args)

    node = Controller()
    rclpy.spin(node)

    rclpy.shutdown()