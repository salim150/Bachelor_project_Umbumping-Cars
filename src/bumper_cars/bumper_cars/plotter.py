#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partialmethod
from custom_message.msg import ControlInputs
import pygame
import math
from matplotlib import pyplot as plt
from custom_message.msg import ControlInputs, State, FullState
import message_filters
import numpy as np

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.5 # [m/s]
        self.min_speed = -0.5# [m/s]
        self.max_delta = np.radians(45)  # [rad]
        self.v_resolution = 0.1# [m/s]
        self.delta_resolution = math.radians(5)  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check
        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]

        self.trail_length = 300

config = Config()
debug = False

class Plotter(Node):

    def __init__(self):
        super().__init__("plotter")
        
        state1_subscriber = message_filters.Subscriber(self, FullState, "/robot1_fullstate")
        state2_subscriber = message_filters.Subscriber(self, FullState, "/robot2_fullstate")

        self.state1_buf = np.array([0,0])
        self.state2_buf = np.array([0,0])

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber], 10, 1, allow_headerless=True)
        ts.registerCallback(self.plotter_callback)
        
        self.get_logger().info("Plotter has been started")

    def plotter_callback(self, state1: FullState, state2: FullState):
    
        self.state1_buf = np.vstack((self.state1_buf, [state1.x, state1.y]))
        self.state2_buf = np.vstack((self.state2_buf, [state2.x, state2.y]))
        if len(self.state1_buf) > config.trail_length:
            self.state1_buf = np.delete(self.state1_buf, 0, 0)
        if len(self.state2_buf) > config.trail_length:
            self.state2_buf = np.delete(self.state2_buf, 0, 0)

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        plt.scatter(self.state1_buf[:,0], self.state1_buf[:,1], linewidths=0.5)
        plt.scatter(self.state2_buf[:,0], self.state2_buf[:,1], linewidths=0.5)
        self.plot_robot(state1)
        self.plot_robot(state2)
        plt.plot(state1.x, state1.y, 'k.')
        plt.plot(state2.x, state2.y, 'b.')
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.000001)

        if debug:
            self.get_logger().info("robot1, x: " + str(state1.x) + ", " +
                                "y: " + str(state1.y) + ", " +
                                "yaw: " + str(state1.yaw) + ", " +
                                "linear velocity: " + str(state1.v))
            self.get_logger().info("robot2, x: " + str(state2.x) + ", " +
                                "y: " + str(state2.y) + ", " +
                                "yaw: " + str(state2.yaw) + ", " +
                                "linear velocity: " + str(state2.v))
        
    def plot_robot(self, fullstate: FullState):
        self.plot_rect(fullstate.x, fullstate.y, fullstate.yaw, config)
        self.plot_cs_robot(fullstate.x, fullstate.y, fullstate.yaw)
        self.plot_arrow(fullstate.x, fullstate.y, fullstate.yaw + fullstate.delta, length=2)

    def plot_arrow(self, x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
    
    def plot_cs_robot(self, x, y, yaw, length=1, width=0.1):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        plt.arrow(x, y, length * -math.sin(yaw), length * math.cos(yaw),
                head_length=width, head_width=width)
        
    def plot_rect(self, x, y, yaw, config):  # pragma: no cover
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                                (config.robot_length / 2), -config.robot_length / 2,
                                -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                                - config.robot_width / 2, -config.robot_width / 2,
                                config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                            [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), "-k")
        


def main(args=None):
    rclpy.init(args=args)

    node = Plotter()
    rclpy.spin(node)

    rclpy.shutdown()

# colcon build --symlink-install to be able to run the node without building it
# or
# colcon build in the source file and the source ~/.bashrc !!!