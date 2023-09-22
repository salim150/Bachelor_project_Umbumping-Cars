#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from custom_message.msg import ControlInputs, State
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
        self.initial_state1 = State()
        self.initial_state1.x = 0.0
        self.initial_state1.y = 0.0
        self.initial_state1.yaw = 0.0
        self.initial_state1.v = 0.0

        self.initial_state2 = State()
        self.initial_state2.x = 0.0
        self.initial_state2.y = 0.0
        self.initial_state2.yaw = 0.0
        self.initial_state2.v = 0.0

        self.state1_publisher_ = self.create_publisher(State, "/robot1_state", 1)
        self.state1_publisher_.publish(self.initial_state1)

        self.state2_publisher_ = self.create_publisher(State, "/robot2_state", 1)
        self.state2_publisher_.publish(self.initial_state2)

        control1_subscriber = message_filters.Subscriber(self, ControlInputs, "/robot1_control")
        control2_subscriber = message_filters.Subscriber(self, ControlInputs, "/robot2_control")

        ts = message_filters.ApproximateTimeSynchronizer([control1_subscriber, control2_subscriber], 2, 0.1, allow_headerless=True)
        ts.registerCallback(self.general_model_callback)

        # self.timer = self.create_timer(0.1, self.update)
        
        #self.get_logger().info(robot_name + "_model started succesfully at position x: " + str(self.initial_state.x) + " , y: " + str(self.initial_state.y))

        self.new_state1 = self.initial_state1
        self.new_state2 = self.initial_state2

        self.old_time1 = time.time()
        self.old_time2 = time.time()

        self.get_logger().info("Robots models initialized correctly")

    def general_model_callback(self, control1: ControlInputs, control2: ControlInputs):

        self.new_state1, self.old_time1 = self.linear_model_callback(self.initial_state1, control1, self.old_time1)
        self.new_state2, self.old_time2 = self.linear_model_callback(self.initial_state2, control2, self.old_time2)

        self.initial_state1 = self.new_state1
        self.initial_state2 = self.new_state2

        

        """self.get_logger().info("Publishing robot1 new state, x: " + str(self.new_state1.x) + ", " +
                               "y: " + str(self.new_state1.y) + ", " +
                               "theta: " + str(self.new_state1.yaw) + ", " +
                               "linear velocity: " + str(self.new_state1.v))"""
        self.state1_publisher_.publish(self.new_state1)
        """self.get_logger().info("Publishing robot2 new state, x: " + str(self.new_state2.x) + ", " +
                               "y: " + str(self.new_state2.y) + ", " +
                               "theta: " + str(self.new_state2.yaw) + ", " +
                               "linear velocity: " + str(self.new_state2.v))"""
        self.state2_publisher_.publish(self.new_state2)

    def linear_model_callback(self, initial_state: State, cmd: ControlInputs, old_time: float):
        
        new_state = State()
        # self.get_logger().info("Command inputs, delta: " + str(cmd.delta) + ",  throttle: " + str(cmd.throttle))

        dt = time.time() - old_time
        print(dt)
        old_time = time.time()
        cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)

        new_state.x += initial_state.v * np.cos(initial_state.yaw) * dt
        new_state.y += initial_state.v * np.sin(initial_state.yaw) * dt
        new_state.yaw += initial_state.v / L * np.tan(cmd.delta) * dt
        new_state.yaw = self.normalize_angle(new_state.yaw)
        new_state.v += cmd.throttle * dt
        new_state.v = np.clip(new_state.v, min_speed, max_speed)

        return new_state, old_time
    
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