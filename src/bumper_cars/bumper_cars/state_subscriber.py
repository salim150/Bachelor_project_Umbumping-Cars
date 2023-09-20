#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from custom_message.msg import ControlInputs, State

class PoseSubscriberNode(Node):
    
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber = self.create_subscription(State,
                                                         "/robot_state", 
                                                         self.pose_callback, 10)


    def pose_callback(self, msg: State):
        self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")")



def main(arg=None):
    rclpy.init(args=arg)
    
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()