#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_message.msg import MultiState, FullState, State
import message_filters
import time

debug = False

class SensorMeasurement(Node):

    def __init__(self):
        super().__init__("sensor")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_type', rclpy.Parameter.Type.STRING_ARRAY),
                ('x0', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('y0', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('yaw', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('v', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('omega', rclpy.Parameter.Type.DOUBLE_ARRAY)
            ]
        )

        x0 = self.get_parameter('x0').get_parameter_value().double_array_value
        y0 = self.get_parameter('y0').get_parameter_value().double_array_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_array_value
        v = self.get_parameter('v').get_parameter_value().double_array_value
        omega = self.get_parameter('omega').get_parameter_value().double_array_value
        model_type = self.get_parameter('model_type').get_parameter_value().string_array_value
        
        # Initializing the robots
        initial_state1 = State(x=x0[0], y=y0[0], yaw=yaw[0], v=v[0], omega=omega[0])
        initial_state2 = State(x=x0[1], y=y0[1], yaw=yaw[1], v=v[1], omega=omega[1])
        initial_state3 = State(x=x0[2], y=y0[2], yaw=yaw[2], v=v[2], omega=omega[2])

        fullstate1 = FullState(x=initial_state1.x, y=initial_state1.y, yaw=initial_state1.yaw, v=initial_state1.v,
                                     omega=initial_state1.omega, delta=0.0, throttle=0.0)
        fullstate2 = FullState(x=initial_state2.x, y=initial_state2.y, yaw=initial_state2.yaw, v=initial_state2.v,
                                     omega=initial_state2.omega, delta=0.0, throttle=0.0)
        fullstate3 = FullState(x=initial_state3.x, y=initial_state3.y, yaw=initial_state3.yaw, v=initial_state3.v,
                                     omega=initial_state3.omega, delta=0.0, throttle=0.0)
        
        self.multi_state = MultiState(multiple_state=[fullstate1, fullstate2, fullstate3])
        self.multi_state_pub = self.create_publisher(MultiState, "/robot_multi_state", 2)
        self.timer = self.create_timer(0.1, self.timer_callback)

        multi_state_subscriber = message_filters.Subscriber(self, MultiState, "/multi_fullstate")

        ts = message_filters.ApproximateTimeSynchronizer([multi_state_subscriber], 6, 1, allow_headerless=True)
        ts.registerCallback(self.sensor_callback)

        self.get_logger().info("Sensor has been started")
        
    def sensor_callback(self, multi_state_in: MultiState):

        # debug_time = time.time()
        
        full_state1 = FullState(x=multi_state_in.multiple_state[0].x, y=multi_state_in.multiple_state[0].y, yaw=multi_state_in.multiple_state[0].yaw,
                                v=multi_state_in.multiple_state[0].v, omega=multi_state_in.multiple_state[0].omega, delta=multi_state_in.multiple_state[0].delta,
                                throttle=multi_state_in.multiple_state[0].throttle)
        full_state2 = FullState(x=multi_state_in.multiple_state[1].x, y=multi_state_in.multiple_state[1].y, yaw=multi_state_in.multiple_state[1].yaw,
                                v=multi_state_in.multiple_state[1].v, omega=multi_state_in.multiple_state[1].omega, delta=multi_state_in.multiple_state[1].delta,
                                throttle=multi_state_in.multiple_state[1].throttle)
        full_state3 = FullState(x=multi_state_in.multiple_state[2].x, y=multi_state_in.multiple_state[2].y, yaw=multi_state_in.multiple_state[2].yaw,
                                v=multi_state_in.multiple_state[2].v, omega=multi_state_in.multiple_state[2].omega, delta=multi_state_in.multiple_state[2].delta,
                                throttle=multi_state_in.multiple_state[2].throttle)

        self.multi_state = MultiState(multiple_state=[full_state1, full_state2, full_state3])
        # self.multi_state_pub.publish(multi_state)

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
        
        # print(time.time()-debug_time)
        # debug_time = time.time()

    def timer_callback(self):
        self.multi_state_pub.publish(self.multi_state)
        
def main(args=None):
    rclpy.init(args=args)

    node = SensorMeasurement()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()