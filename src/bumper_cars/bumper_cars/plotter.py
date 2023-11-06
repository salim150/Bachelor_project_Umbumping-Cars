#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from matplotlib import pyplot as plt
from custom_message.msg import FullState, Path, MultiplePaths, MultiState
import message_filters
import numpy as np
import time

# For the parameter file
import pathlib
import json

path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        self.robot_width = 1.45  # [m] for collision check
        self.robot_length = 2.9  # [m] for collision check
        # obstacles [x(m) y(m), ....]

        self.trail_length = 5

config = Config()
controller_type = json_object["Controller"]["controller_type"]
debug = False
plot_traj = True

class Plotter(Node):

    def __init__(self):
        super().__init__("plotter")

        multi_state_sub = message_filters.Subscriber(self, MultiState, "/multi_fullstate")
        self.multi_path_sub = message_filters.Subscriber(self, MultiplePaths, "/robot_multi_traj")

        self.state1_buf = np.array([0,0])
        self.state2_buf = np.array([0,0])
        self.state3_buf = np.array([0,0])

        # TODO: Pass it from parameters file
        self.controller_type = controller_type

        self.width = 100
        self.heigth = 100

        if self.controller_type == "random_walk":
            ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub], 10, 1, allow_headerless=True)
            ts.registerCallback(self.plotter_callback)
        else:
            ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub, self.multi_path_sub], 10, 1, allow_headerless=True)
            ts.registerCallback(self.complete_plotter_callback)

        self.get_logger().info("Plotter has been started")

    def plotter_callback(self, multi_state: MultiState):
       
        state1 = multi_state.multiple_state[0]
        state2 = multi_state.multiple_state[1]
        state3 = multi_state.multiple_state[2]
        
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        self.plot_robot(state1)
        self.plot_robot(state2)
        self.plot_robot(state3)

        self.plot_map()
        plt.axis("equal")
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
            self.get_logger().info("robot3, x: " + str(state3.x) + ", " +
                                "y: " + str(state3.y) + ", " +
                                "yaw: " + str(state3.yaw) + ", " +
                                "linear velocity: " + str(state3.v))

    def complete_plotter_callback(self, multi_state: MultiState, multi_traj: MultiplePaths):
        # debug_time = time.time()

        state1 = multi_state.multiple_state[0]
        state2 = multi_state.multiple_state[1]
        state3 = multi_state.multiple_state[2]
    
        self.state1_buf = np.vstack((self.state1_buf, [state1.x, state1.y]))
        self.state2_buf = np.vstack((self.state2_buf, [state2.x, state2.y]))
        self.state3_buf = np.vstack((self.state3_buf, [state3.x, state3.y]))

        if len(self.state1_buf) > config.trail_length:
            self.state1_buf = np.delete(self.state1_buf, 0, 0)
        if len(self.state2_buf) > config.trail_length:
            self.state2_buf = np.delete(self.state2_buf, 0, 0)
        if len(self.state3_buf) > config.trail_length:
            self.state3_buf = np.delete(self.state3_buf, 0, 0)
        
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        self.plot_situation(state1, multi_traj.multiple_path[0], self.state1_buf)
        self.plot_situation(state2, multi_traj.multiple_path[1], self.state2_buf)
        self.plot_situation(state3, multi_traj.multiple_path[2], self.state3_buf)

        self.plot_map()
        plt.axis("equal")
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
            self.get_logger().info("robot3, x: " + str(state3.x) + ", " +
                                "y: " + str(state3.y) + ", " +
                                "yaw: " + str(state3.yaw) + ", " +
                                "linear velocity: " + str(state3.v))
            
        # print(time.time()-debug_time)
        # debug_time = time.time()

    def plot_situation(self, state: FullState, trajectory, state_buf):
        if plot_traj:
            self.plot_path(trajectory)
            plt.scatter(state_buf[:,0], state_buf[:,1], marker='.', s=4)
            plt.plot(state.x, state.y, 'b.')
        self.plot_robot(state)


    def plot_robot(self, fullstate: FullState):
        self.plot_rect(fullstate.x, fullstate.y, fullstate.yaw, config)
        self.plot_cs_robot(fullstate.x, fullstate.y, fullstate.yaw, length=2)
        self.plot_arrow(fullstate.x, fullstate.y, fullstate.yaw + fullstate.delta, length=4)

    def plot_map(self):
        corner_x = [-self.width/2.0, self.width/2.0, self.width/2.0, -self.width/2.0, -self.width/2.0]
        corner_y = [self.heigth/2.0, self.heigth/2.0, -self.heigth/2.0, -self.heigth/2.0, self.heigth/2.0]

        plt.plot(corner_x, corner_y)

    def plot_path(self, path: Path):
        x = []
        y = []
        for coord in path.path:
            x.append(coord.x)
            y.append(coord.y)
        plt.scatter(x, y, marker='.', s=10)
        plt.scatter(x[0], y[0], marker='x', s=20)


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

    node.destroy_node()
    rclpy.shutdown()

# colcon build --symlink-install to be able to run the node without building it
# or
# colcon build in the source file and the source ~/.bashrc !!!