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

class SensorMeasurement(Node):

    def __init__(self):
        super().__init__("sensor")

        self.measurement1_publisher_ = self.create_publisher(State, "/robot1_measurement", 20)
        self.measurement2_publisher_ = self.create_publisher(State, "/robot2_measurement", 20)
        self.measurement3_publisher_ = self.create_publisher(State, "/robot3_measurement", 20)
        self.measurement4_publisher_ = self.create_publisher(State, "/robot4_measurement", 20)
        
        state1_subscriber = message_filters.Subscriber(self, State, "/robot1_state")
        state2_subscriber = message_filters.Subscriber(self, State, "/robot2_state")
        state3_subscriber = message_filters.Subscriber(self, State, "/robot3_state")
        state4_subscriber = message_filters.Subscriber(self, State, "/robot4_state")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber, state3_subscriber, state4_subscriber], 6, 1, allow_headerless=True)
        ts.registerCallback(self.sensor_callback)

        self.get_logger().info("Sensor has been started")
        
    def sensor_callback(self, state1: State, state2: State, state3: State, state4: State):

        self.measurement1_publisher_.publish(state1)
        self.measurement2_publisher_.publish(state2)
        self.measurement3_publisher_.publish(state3)
        self.measurement4_publisher_.publish(state4)

        if debug:
            self.get_logger().info("Publishing robot1 new state, x: " + str(state1.x) + ", " +
                                "y: " + str(state1.y) + ", " +
                                "theta: " + str(state1.yaw) + ", " +
                                "linear velocity: " + str(state1.v))
            self.get_logger().info("Publishing robot2 new state, x: " + str(state2.x) + ", " +
                                "y: " + str(state2.y) + ", " +
                                "theta: " + str(state2.yaw) + ", " +
                                "linear velocity: " + str(state2.v))
            self.get_logger().info("Publishing robot3 new state, x: " + str(state3.x) + ", " +
                                "y: " + str(state3.y) + ", " +
                                "theta: " + str(state3.yaw) + ", " +
                                "linear velocity: " + str(state3.v))
            self.get_logger().info("Publishing robot3 new state, x: " + str(state4.x) + ", " +
                                "y: " + str(state4.y) + ", " +
                                "theta: " + str(state4.yaw) + ", " +
                                "linear velocity: " + str(state4.v))

def main(args=None):
    rclpy.init(args=args)

    node = SensorMeasurement()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()