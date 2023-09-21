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


class Plotter(Node):

    def __init__(self, robot_name: str, color: str):
        super().__init__(robot_name + "_plotter")
        self.get_logger().info("Plotter has been started")
        
        self.plotter_subscriber_ = self.create_subscription(State, "/" + robot_name + "_state", self.plotter_callback, 10)
        self.color = color
        
    def plotter_callback(self, pose: State):
        
        msg = State()

        msg.x = pose.x
        msg.y = pose.y
        msg.yaw = pose.yaw
        msg.v = pose.v
        
        # add arrow to put in the direction, make plotting fancier
        plt.figure()
        plt.plot(msg.x, msg.y, self.color)
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

        self.get_logger().info("x: " + str(msg.x) + ", " +
                               "y: " + str(msg.y) + ", " +
                               "theta: " + str(msg.yaw) + ", " +
                               "linear velocity: " + str(msg.v))



def main(args=None):
    rclpy.init(args=args)

    node1 = Plotter("robot1", 'k.')
    node2 = Plotter("robot2", 'g.')
    rclpy.spin(node1)
    rclpy.spin(node2)

    rclpy.shutdown()

# colcon build --symlink-install to be able to run the node without building it
# or
# colcon build in the source file and the source ~/.bashrc !!!