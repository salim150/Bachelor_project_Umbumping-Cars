#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_message.msg import ControlInputs, State, FullState, MultiControl, MultiState
import numpy as np
import time
import message_filters
import math

max_steer = np.radians(30.0)  # [rad] max steering angle
max_speed = 10 # [m/s]
min_speed = 0.0 # [m/s]
L = 2.9  # [m] Wheel base of vehicle
Lr = L / 2.0  # [m]
Lf = L - Lr
Cf = 160.0 * 2.0  # N/rad
Cr = 170.0 * 2.0  # N/rad
Iz = 225.0  # kg/m2
m = 150.0  # kg
# Aerodynamic and friction coefficients
c_a = 1.36
c_r1 = 0.01


class CarModel(Node):

    def __init__(self):
        super().__init__("robot_model")

        # TODO: pass parameters as State type and not single items --> USE ARRAYS/DICT   
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

        self.x0 = self.get_parameter('x0').get_parameter_value().double_array_value
        self.y0 = self.get_parameter('y0').get_parameter_value().double_array_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_array_value
        self.v = self.get_parameter('v').get_parameter_value().double_array_value
        self.omega = self.get_parameter('omega').get_parameter_value().double_array_value
        self.model_type = self.get_parameter('model_type').get_parameter_value().string_array_value

        # Initializing the robots
        self.initial_state1 = State(x=self.x0[0], y=self.y0[0], yaw=self.yaw[0], v=self.v[0], omega=self.omega[0])
        self.initial_state2 = State(x=self.x0[1], y=self.y0[1], yaw=self.yaw[1], v=self.v[1], omega=self.omega[1])
        self.initial_state3 = State(x=self.x0[2], y=self.y0[2], yaw=self.yaw[2], v=self.v[2], omega=self.omega[2])

        # Initializing the state publishers/subscribers
        self.control_sub = self.create_subscription(MultiControl, '/robot_control', self.general_model_callback, 10)
        self.fullstate_publisher_ = self.create_publisher(MultiState, "robot_fullstate", 60)

        self.old_time1 = time.time()
        self.old_time2 = time.time()
        self.old_time3 = time.time()
        self.old_time4 = time.time()

        self.get_logger().info("Robots model initialized correctly")

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.fullstate1 = FullState(x=self.initial_state1.x, y=self.initial_state1.y, yaw=self.initial_state1.yaw, v=self.initial_state1.v,
                                     omega=self.initial_state1.omega, delta=0.0, throttle=0.0)
        self.fullstate2 = FullState(x=self.initial_state2.x, y=self.initial_state2.y, yaw=self.initial_state2.yaw, v=self.initial_state2.v,
                                     omega=self.initial_state2.omega, delta=0.0, throttle=0.0)
        self.fullstate3 = FullState(x=self.initial_state3.x, y=self.initial_state3.y, yaw=self.initial_state3.yaw, v=self.initial_state3.v,
                                     omega=self.initial_state3.omega, delta=0.0, throttle=0.0)
        
        self.multi_state = MultiState(multiple_state=[self.fullstate1, self.fullstate2, self.fullstate3])

    def general_model_callback(self, control: MultiControl):


        if self.model_type[0] == 'linear':
            self.initial_state1, self.old_time1 = self.linear_model_callback(self.initial_state1, control.multi_control[0], self.old_time1)
        elif self.model_type[0] == 'nonlinear':
            self.initial_state1, self.old_time1 = self.nonlinear_model_callback(self.initial_state1, control.multi_control[0], self.old_time1)

        if self.model_type[1] == 'linear':
            self.initial_state2, self.old_time2 = self.linear_model_callback(self.initial_state2, control.multi_control[1], self.old_time2)
        elif self.model_type[1] == 'nonlinear':
            self.initial_state2, self.old_time2 = self.nonlinear_model_callback(self.initial_state2, control.multi_control[1], self.old_time2)
        
        if self.model_type[2] == 'linear':
            self.initial_state3, self.old_time3 = self.linear_model_callback(self.initial_state3, control.multi_control[2], self.old_time3)
        elif self.model_type[2] == 'nonlinear':
            self.initial_state3, self.old_time3 = self.nonlinear_model_callback(self.initial_state3, control.multi_control[2], self.old_time3)
        
        self.fullstate1 = FullState(x=float(self.initial_state1.x), y=float(self.initial_state1.y), yaw=float(self.initial_state1.yaw), v=float(self.initial_state1.v),
                               omega=float(self.initial_state1.omega), delta=float(control.multi_control[0].delta), throttle=float(control.multi_control[0].throttle))
        
        self.fullstate2 = FullState(x=float(self.initial_state2.x), y=float(self.initial_state2.y), yaw=float(self.initial_state2.yaw), v=float(self.initial_state2.v),
                               omega=float(self.initial_state2.omega), delta=float(control.multi_control[1].delta), throttle=float(control.multi_control[1].throttle))
        
        self.fullstate3 = FullState(x=float(self.initial_state3.x), y=float(self.initial_state3.y), yaw=float(self.initial_state3.yaw), v=float(self.initial_state3.v),
                               omega=float(self.initial_state3.omega), delta=float(control.multi_control[2].delta), throttle=float(control.multi_control[2].throttle))
        
        self.multi_state = MultiState(multiple_state=[self.fullstate1, self.fullstate2, self.fullstate3])

    def linear_model_callback(self, initial_state: State, cmd: ControlInputs, old_time: float):

        dt = time.time() - old_time
        state = State()
        cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)

        state.x = initial_state.x + initial_state.v * np.cos(initial_state.yaw) * dt
        state.y = initial_state.y + initial_state.v * np.sin(initial_state.yaw) * dt
        state.yaw = initial_state.yaw + initial_state.v / L * np.tan(cmd.delta) * dt
        state.yaw = self.normalize_angle(state.yaw)
        state.v = initial_state.v + cmd.throttle * dt
        state.v = np.clip(state.v, min_speed, max_speed)

        return state, time.time()
    
    def nonlinear_model_callback(self, state: State, cmd: ControlInputs, old_time: float):

        dt = time.time() - old_time
        cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)

        beta = math.atan2((Lr * math.tan(cmd.delta) / L), 1.0)
        vx = state.v * math.cos(beta)
        vy = state.v * math.sin(beta)

        Ffy = -Cf * ((vy + Lf * state.omega) / (vx + 0.0001) - cmd.delta)
        Fry = -Cr * (vy - Lr * state.omega) / (vx + 0.0001)
        R_x = c_r1 * abs(vx)
        F_aero = c_a * vx ** 2 # 
        F_load = F_aero + R_x #
        state.omega = state.omega + (Ffy * Lf * math.cos(cmd.delta) - Fry * Lr) / Iz * dt
        vx = vx + (cmd.throttle - Ffy * math.sin(cmd.delta) / m - F_load / m + vy * state.omega) * dt
        vy = vy + (Fry / m + Ffy * math.cos(cmd.delta) / m - vx * state.omega) * dt

        state.yaw = state.yaw + state.omega * dt
        state.yaw = self.normalize_angle(state.yaw)

        state.x = state.x + vx * math.cos(state.yaw) * dt - vy * math.sin(state.yaw) * dt
        state.y = state.y + vx * math.sin(state.yaw) * dt + vy * math.cos(state.yaw) * dt

        state.v = math.sqrt(vx ** 2 + vy ** 2)
        state.v = np.clip(state.v, min_speed, max_speed)

        return state, time.time()
    
    def timer_callback(self):
        #self.state1_publisher_.publish(self.initial_state1)
        self.fullstate_publisher_.publish(self.multi_state)

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

    node.destroy_node()
    rclpy.shutdown()