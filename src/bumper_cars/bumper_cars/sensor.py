#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs

class SensorMeasurement(Node):

    def __init__(self):
        super().__init__("sensor")
        self.get_logger().info("Sensor has been started")

        self.measurement_publisher_ = self.create_publisher(Pose, "/sensor_measurement", 10)
        self.pose_subscriber_ = self.create_subscription(Pose,
                                                         "/turtle1/pose", 
                                                         self.pose_callback, 10) # replace with topic /robot_state
        
    def pose_callback(self, pose: Pose):
        
        msg = Pose()

        msg.x = pose.x
        msg.y = pose.y
        msg.theta = pose.theta
        msg.linear_velocity = pose.linear_velocity
        msg.angular_velocity = pose.angular_velocity

        self.measurement_publisher_.publish(msg)
        self.get_logger().info("x: " + str(msg.x) + ", " +
                               "y: " + str(msg.y) + ", " +
                               "theta: " + str(msg.theta) + ", " +
                               "linear velocity: " + str(msg.linear_velocity) + ", " +
                               "angular velocity: " + str(msg.angular_velocity))



def main(args=None):
    rclpy.init(args=args)

    node = SensorMeasurement()
    rclpy.spin(node)

    rclpy.shutdown()