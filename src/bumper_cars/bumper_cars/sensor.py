#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State

class SensorMeasurement(Node):

    def __init__(self, robot_name: str):
        super().__init__(robot_name + "_sensor")
        self.get_logger().info("Sensor has been started")

        self.measurement_publisher_ = self.create_publisher(State, "/" + robot_name + "_measurement", 10)
        self.pose_subscriber_ = self.create_subscription(State,
                                                         "/" + robot_name + "_state", 
                                                         self.pose_callback, 10) # replace with topic /robot_state
        
    def pose_callback(self, pose: State):
        
        msg = State()

        msg.x = pose.x
        msg.y = pose.y
        msg.yaw = pose.yaw
        msg.v = pose.v

        self.measurement_publisher_.publish(msg)
        self.get_logger().info("x: " + str(msg.x) + ", " +
                               "y: " + str(msg.y) + ", " +
                               "yaw: " + str(msg.yaw) + ", " +
                               "linear velocity: " + str(msg.v))



def main(args=None):
    rclpy.init(args=args)

    node1 = SensorMeasurement("robot1")
    node2 = SensorMeasurement("robot2")
    rclpy.spin(node1)
    rclpy.spin(node2)

    rclpy.shutdown()