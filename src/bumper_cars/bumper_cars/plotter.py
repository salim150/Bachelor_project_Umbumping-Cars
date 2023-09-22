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
from custom_message.msg import ControlInputs, State
import message_filters


class Plotter(Node):

    def __init__(self):
        super().__init__("plotter")
        self.get_logger().info("Plotter has been started")
        
        # self.plotter_subscriber_ = self.create_subscription(State, "/" + robot_name + "_state", self.plotter_callback, 1)
        state1_subscriber = message_filters.Subscriber(self, State, "/robot1_state")
        state2_subscriber = message_filters.Subscriber(self, State, "/robot2_state")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber], 10, 1, allow_headerless=True)
        ts.registerCallback(self.plotter_callback)
        
    def plotter_callback(self, state1: State, state2: State):
        
        # add arrow to put in the direction, make plotting fancier
        plt.plot(state1.x, state1.y, 'k.')
        plt.plot(state2.x, state2.y, 'b.')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

        self.get_logger().info("robot1, x: " + str(state1.x) + ", " +
                               "y: " + str(state1.y) + ", " +
                               "theta: " + str(state1.yaw) + ", " +
                               "linear velocity: " + str(state1.v))
        self.get_logger().info("robot2, x: " + str(state2.x) + ", " +
                               "y: " + str(state2.y) + ", " +
                               "theta: " + str(state2.yaw) + ", " +
                               "linear velocity: " + str(state2.v))



def main(args=None):
    rclpy.init(args=args)

    node = Plotter()
    rclpy.spin(node)

    rclpy.shutdown()

# colcon build --symlink-install to be able to run the node without building it
# or
# colcon build in the source file and the source ~/.bashrc !!!