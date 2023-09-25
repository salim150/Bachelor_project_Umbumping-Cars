#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from custom_message.msg import ControlInputs, State, FullState
import numpy as np
import time
import message_filters

max_steer = np.radians(30.0)  # [rad] max steering angle
max_speed = 5 # [m/s]
min_speed = 0 # [m/s]
L = 2.9  # [m] Wheel base of vehicle
# dt = 0.1
Lr = L / 2.0  # [m]
Lf = L - Lr
Cf = 1600.0 * 2.0  # N/rad
Cr = 1700.0 * 2.0  # N/rad
Iz = 2250.0  # kg/m2
m = 1500.0  # kg


class CarModel(Node):

    def __init__(self):
        super().__init__("robot_models")

        # Initializing the robot
        self.initial_state1 = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
        self.initial_state2 = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

        self.state1_publisher_ = self.create_publisher(State, "/robot1_state", 1)
        self.state2_publisher_ = self.create_publisher(State, "/robot2_state", 1)

        self.control1_subscriber = message_filters.Subscriber(self, ControlInputs, "/robot1_control")
        self.control2_subscriber = message_filters.Subscriber(self, ControlInputs, "/robot2_control")

        self.fullstate1_publisher_ = self.create_publisher(FullState, "robot1_fullstate", 1)
        self.fullstate2_publisher_ = self.create_publisher(FullState, "robot2_fullstate", 1)

        ts = message_filters.ApproximateTimeSynchronizer([self.control1_subscriber, self.control2_subscriber], 2, 0.1, allow_headerless=True)
        ts.registerCallback(self.general_model_callback)

        self.old_time1 = time.time()
        self.old_time2 = time.time()

        self.state1_publisher_.publish(self.initial_state1)
        self.state2_publisher_.publish(self.initial_state2)
        self.get_logger().info("Robots models initialized correctly")

    def general_model_callback(self, control1: ControlInputs, control2: ControlInputs):

        self.initial_state1, self.old_time1 = self.linear_model_callback(self.initial_state1, control1, self.old_time1)
        self.initial_state2, self.old_time2 = self.linear_model_callback(self.initial_state2, control2, self.old_time2)

        self.state1_publisher_.publish(self.initial_state1)
        self.state2_publisher_.publish(self.initial_state2)
        
        fullstate1 = FullState(x=float(self.initial_state1.x), y=float(self.initial_state1.y), yaw=float(self.initial_state1.yaw), v=float(self.initial_state1.v),
                                                     delta=float(control1.delta), throttle=float(control1.throttle))
        self.fullstate1_publisher_.publish(fullstate1)
        fullstate2 = FullState(x=float(self.initial_state2.x), y=float(self.initial_state2.y), yaw=float(self.initial_state2.yaw), v=float(self.initial_state2.v),
                                                     delta=float(control2.delta), throttle=float(control2.throttle))
        self.fullstate2_publisher_.publish(fullstate2)

    def linear_model_callback(self, state: State, cmd: ControlInputs, old_time: float):
        
        new_state = State()

        dt = time.time() - old_time
        #print(dt)
        cmd.delta = np.clip(np.radians(cmd.delta), -max_steer, max_steer)

        state.x += state.v * np.cos(state.yaw) * dt
        state.y += state.v * np.sin(state.yaw) * dt
        state.yaw += state.v / L * np.tan(cmd.delta) * dt
        state.yaw = self.normalize_angle(state.yaw)
        state.v += cmd.throttle * dt
        state.v = np.clip(state.v, min_speed, max_speed)

        return state, time.time()
    
    """def update(self):
        self.state_publisher_.publish(self.new_state)
        self.get_logger().info("Publishing state: " + self.robot_name)
        self.initial_state = self.new_state"""

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle
        
        
def main(args=None):
    rclpy.init(args=args)
    
    node = CarModel()
    rclpy.spin(node)

    rclpy.shutdown()