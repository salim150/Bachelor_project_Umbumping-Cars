#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from custom_message.msg import ControlInputs, State
import numpy as np
import time

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

    def __init__(self, robot_name: str):
        super().__init__(robot_name + "_model")

        # Initializing the robot
        self.initial_state = State()
        self.initial_state.x = 0.0
        self.initial_state.y = 0.0
        self.initial_state.yaw = 0.0
        self.initial_state.v = 0.0

        self.state_publisher_ = self.create_publisher(State, "/" + robot_name + "_state", 10)
        self.state_publisher_.publish(self.initial_state)

        """self.state_subscriber_ = self.create_subscription(State,
                                                         "/robot_state", self.model_callback, 10)""" 
        self.control_subscriber_ = self.create_subscription(ControlInputs,
                                                         "/" + robot_name + "_control", 
                                                         self.model_callback, 10) 
        #self.timer = self.create_timer(0.1, self.update)
        
        self.get_logger().info("Car model started succesfully at position x: " + str(self.initial_state.x) + " , y: " + str(self.initial_state.y))

        self.new_state = self.initial_state
        self.old_time = time.time()

    def model_callback(self, cmd: ControlInputs):
        
        self.get_logger().info("Command inputs, delta: " + str(cmd.delta) + ",  throttle: " + str(cmd.throttle))


        dt = time.time() - self.old_time
        self.old_time = time.time()
        # dt = 0.1
        cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)

        self.new_state.x += self.initial_state.v * np.cos(self.initial_state.yaw) * dt
        self.new_state.y += self.initial_state.v * np.sin(self.initial_state.yaw) * dt
        self.new_state.yaw += self.initial_state.v / L * np.tan(cmd.delta) * dt
        print(f'Yaw: {self.new_state.yaw}')
        self.new_state.yaw = self.normalize_angle(self.new_state.yaw)
        self.new_state.v += cmd.throttle * dt
        self.new_state.v = np.clip(self.new_state.v, min_speed, max_speed)

        self.state_publisher_.publish(self.new_state)
    
    def update(self):
        self.state_publisher_.publish(self.new_state)
        self.get_logger().info("Publishing state")
        self.initial_state = self.new_state

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
    
    # maybe add initialization of the robots here in the main
    node1 = CarModel("robot1")
    node2 = CarModel("robot2")
    rclpy.spin(node1)
    rclpy.spin(node2)

    rclpy.shutdown()