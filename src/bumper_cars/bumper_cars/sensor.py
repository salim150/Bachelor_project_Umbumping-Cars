#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State, MultiState, FullState
import message_filters

debug = False

class SensorMeasurement(Node):

    def __init__(self):
        super().__init__("sensor")

        self.multi_state_pub = self.create_publisher(MultiState, "/robot_multi_state", 2)
        
        state1_subscriber = message_filters.Subscriber(self, FullState, "/robot1_fullstate")
        state2_subscriber = message_filters.Subscriber(self, FullState, "/robot2_fullstate")
        state3_subscriber = message_filters.Subscriber(self, FullState, "/robot3_fullstate")
        state4_subscriber = message_filters.Subscriber(self, FullState, "/robot4_fullstate")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber, state3_subscriber, state4_subscriber], 6, 1, allow_headerless=True)
        ts.registerCallback(self.sensor_callback)

        self.get_logger().info("Sensor has been started")
        
    def sensor_callback(self, full_state1: FullState, full_state2: FullState, full_state3: FullState, full_state4: FullState):

        multi_state = MultiState(multiple_state=[full_state1, full_state2, full_state3, full_state4])
        self.multi_state_pub.publish(multi_state)

        if debug:
            self.get_logger().info("Publishing robot1 new state, x: " + str(full_state1.x) + ", " +
                                "y: " + str(full_state1.y) + ", " +
                                "theta: " + str(full_state1.yaw) + ", " +
                                "linear velocity: " + str(full_state1.v))
            self.get_logger().info("Publishing robot2 new state, x: " + str(full_state2.x) + ", " +
                                "y: " + str(full_state2.y) + ", " +
                                "theta: " + str(full_state2.yaw) + ", " +
                                "linear velocity: " + str(full_state2.v))
            self.get_logger().info("Publishing robot3 new state, x: " + str(full_state3.x) + ", " +
                                "y: " + str(full_state3.y) + ", " +
                                "theta: " + str(full_state3.yaw) + ", " +
                                "linear velocity: " + str(full_state3.v))
            self.get_logger().info("Publishing robot3 new state, x: " + str(full_state4.x) + ", " +
                                "y: " + str(full_state4.y) + ", " +
                                "theta: " + str(full_state4.yaw) + ", " +
                                "linear velocity: " + str(full_state4.v))

def main(args=None):
    rclpy.init(args=args)

    node = SensorMeasurement()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()