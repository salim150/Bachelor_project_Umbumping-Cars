#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from custom_message.msg import ControlInputs, State
import numpy as np

max_steer = np.radians(30.0)  # [rad] max steering angle
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
        super().__init__("car_model")

        self.measurement_publisher_ = self.create_publisher(State, "/robot_state", 10)
        self.state_subscriber_ = self.create_subscription(State,
                                                         "/robot_state", qos_profile=10) 
        self.control_subscriber_ = self.create_subscription(ControlInputs,
                                                         "/robot_control", 
                                                         self.model_callback, 10) 
        # TODO: change the topics name and create new ones. the subscriber of this class should be subscribed to the /control_input topic and publishing the 
        # /current_pose topic
        
        self.get_logger().info("Car model started succesfully")
        
    def model_callback(self, pose: State, cmd: ControlInputs):
        new_state = State()

        dt = 0.1
        cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)

        new_state.x += pose.v * np.cos(pose.yaw) * dt
        new_state.y += pose.v * np.sin(pose.yaw) * dt
        new_state.yaw += pose.v / L * np.tan(cmd.delta) * dt
        new_state.yaw = self.normalize_angle(pose.yaw)
        new_state.v += cmd.throttle * dt

    def normalize_angle(angle):
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