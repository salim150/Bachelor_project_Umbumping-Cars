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

        multi_state_subscriber = message_filters.Subscriber(self, MultiState, "/multi_fullstate")

        ts = message_filters.ApproximateTimeSynchronizer([multi_state_subscriber], 6, 1, allow_headerless=True)
        ts.registerCallback(self.sensor_callback)

        self.get_logger().info("Sensor has been started")
        
    def sensor_callback(self, multi_state_in: MultiState):
        
        full_state1 = FullState(x=multi_state_in.multiple_state[0].x, y=multi_state_in.multiple_state[0].y, yaw=multi_state_in.multiple_state[0].yaw,
                                v=multi_state_in.multiple_state[0].v, omega=multi_state_in.multiple_state[0].omega, delta=multi_state_in.multiple_state[0].delta,
                                throttle=multi_state_in.multiple_state[0].throttle)
        full_state2 = FullState(x=multi_state_in.multiple_state[1].x, y=multi_state_in.multiple_state[1].y, yaw=multi_state_in.multiple_state[1].yaw,
                                v=multi_state_in.multiple_state[1].v, omega=multi_state_in.multiple_state[1].omega, delta=multi_state_in.multiple_state[1].delta,
                                throttle=multi_state_in.multiple_state[1].throttle)
        full_state3 = FullState(x=multi_state_in.multiple_state[2].x, y=multi_state_in.multiple_state[2].y, yaw=multi_state_in.multiple_state[2].yaw,
                                v=multi_state_in.multiple_state[2].v, omega=multi_state_in.multiple_state[2].omega, delta=multi_state_in.multiple_state[2].delta,
                                throttle=multi_state_in.multiple_state[2].throttle)

        multi_state = MultiState(multiple_state=[full_state1, full_state2, full_state3])
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

def main(args=None):
    rclpy.init(args=args)

    node = SensorMeasurement()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()