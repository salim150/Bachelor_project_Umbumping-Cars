#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_message.msg import ControlInputs, State, Path, Coordinate, MultiplePaths, MultiState, MultiControl
import message_filters
import random
import math
import numpy as np
# from planner.cubic_spline_planner import *
# from planner.frenet import *
from planner.predict_traj import *

# for the CBF
from planner.CBF_robotarium import *
from planner.CBF_simple import *
from planner.C3BF import *

# For the parameter file
import pathlib
import json

path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

WB = json_object["Controller"]["WB"] # Wheel base
L_d = json_object["Controller"]["L_d"]  # [m] look-ahead distance
max_steer = json_object["Controller"]["max_steer"]  # [rad] max steering angle

debug = False

class Controller(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.previous_x = 0
        
        # creating random walk path for car2 to follow
        self.safety = 20 # safety border around the map boundaries
        self.width = 100.0 - self.safety
        self.heigth = 100.0 -self.safety

        # Robot 1
        self.path1 = []
        self.path1 = self.create_path(self.path1)
        self.target1 = (self.path1[0].x, self.path1[0].y)
        self.trajectory1, self.tx1, self.ty1 = predict_trajectory(State(x=30.0, y=30.0, yaw=0.0, v=0.0, omega=0.0), self.target1)

        # Robot 2
        self.path2 = []
        self.path2 = self.create_path(self.path2)
        self.target2 = (self.path2[0].x, self.path2[0].y)
        self.trajectory2, self.tx2, self.ty2 = predict_trajectory(State(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0), self.target2)

        # Robot 3
        self.path3 = []
        self.path3 = self.create_path(self.path3)
        self.target3 = (self.path3[0].x, self.path3[0].y)
        self.trajectory3, self.tx3, self.ty3 = predict_trajectory(State(x=-30.0, y=-30.0, yaw=0.0, v=0.0, omega=0.0), self.target3)


        self.multi_control_pub = self.create_publisher(MultiControl, '/multi_control', 20)
        self.multi_path_pub = self.create_publisher(MultiplePaths, "/robot_multi_traj", 2)
        multi_state_sub = message_filters.Subscriber(self, MultiState, "/robot_multi_state")


        ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub], 4, 0.3, allow_headerless=True)
        ts.registerCallback(self.general_pose_callback)

        self.multi_control_pub.publish(MultiControl(multi_control=[ControlInputs(delta=0.0, throttle=0.0), ControlInputs(delta=0.0, throttle=0.0), 
                                                                   ControlInputs(delta=0.0, throttle=0.0)]))
        multi_path = MultiplePaths(multiple_path=[Path(path=self.trajectory1), Path(path=self.trajectory2), Path(path=self.trajectory3)])
        self.multi_path_pub.publish(multi_path)  
     
        # CBF
        self.uni_barrier_cert = create_unicycle_barrier_certificate_with_boundary()

        self.get_logger().info("Controller has been started")

    def general_pose_callback(self, multi_state):

        state1 = multi_state.multiple_state[0]
        state2 = multi_state.multiple_state[1]
        state3 = multi_state.multiple_state[2]

        cmd1, self.path1, self.target1, self.trajectory1 = self.control_callback(state1, self.target1, self.path1, self.trajectory1)
        cmd2, self.path2, self.target2, self.trajectory2 = self.control_callback(state2, self.target2, self.path2, self.trajectory2)
        cmd3, self.path3, self.target3, self.trajectory3 = self.control_callback(state3, self.target3, self.path3, self.trajectory3)

        if debug:
           self.get_logger().info(f'Commands before: cmd1: {cmd1}, cmd2: {cmd2}')
        
        dxu = np.zeros((2,3))
        dxu[0,0], dxu[1,0] = cmd1.throttle, cmd1.delta
        dxu[0,1], dxu[1,1] = cmd2.throttle, cmd2.delta
        dxu[0,2], dxu[1,2] = cmd3.throttle, cmd3.delta

        # Converting positions to arrays for the CBF
        x1 = state_to_array(state1)
        x2 = state_to_array(state2)
        x3 = state_to_array(state3)
        x = np.concatenate((x1, x2, x3), axis=1)

        # Create safe control inputs (i.e., no collisions)
        dxu = self.uni_barrier_cert(dxu, x)
        # dxu = CBF(x, dxu)
        # dxu = C3BF(x, dxu)

        cmd1.throttle, cmd1.delta = dxu[0,0], dxu[1,0]
        cmd2.throttle, cmd2.delta = dxu[0,1], dxu[1,1]
        cmd3.throttle, cmd3.delta = dxu[0,2], dxu[1,2]

        # Publishing everything in the general callback to avoid deadlocks
        multi_control = MultiControl(multi_control=[cmd1, cmd2, cmd3])
        self.multi_control_pub.publish(multi_control)

        multi_path = MultiplePaths(multiple_path=[Path(path=self.trajectory1), Path(path=self.trajectory2), Path(path=self.trajectory3)])
        self.multi_path_pub.publish(multi_path)     
        
        if debug:
            self.get_logger().info(f'Commands after: cmd1: {cmd1}, cmd2: {cmd2}')

    def control_callback(self, pose: FullState, target, path, trajectory):
        # updating target waypoint and predicting new traj
        if self.dist(point1=(pose.x, pose.y), point2=target) < L_d:
            path = self.update_path(path)
            target = (path[0].x, path[0].y)
            trajectory, tx, ty  = predict_trajectory(pose, target)
    
        cmd = ControlInputs()
        cmd.throttle, cmd.delta, path, target = self.pure_pursuit_steer_control(target, pose, path)

        if debug:
            self.get_logger().info("Control input robot1, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))

        return cmd, path, target, trajectory
    
    def update_path(self, path: Path):
        path.pop(0)
        path.append(Coordinate(x=float(random.randint(-self.width/2, self.width/2)), y=float(random.randint(-self.heigth/2, self.heigth/2))))
        return path

    def plot_path(self, path: Path):
        x = []
        y = []
        for coord in path:
            x.append(coord.x)
            y.append(coord.y)
        plt.scatter(x, y, marker='.', s=10)
        plt.scatter(x[0], y[0], marker='x', s=20)

    def create_path(self, path):
        while len(path)<5:
            path.append(Coordinate(x=float(random.randint(-self.width/2, self.width/2)), y=float(random.randint(-self.heigth/2, self.heigth/2))))
        return path
    
    def pure_pursuit_steer_control(self, target, pose: FullState, path: Path):

        alpha = self.normalize_angle(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)

        # this if/else condition should fix the buf of the waypoint behind the car
        if alpha > np.pi/2.0:
            delta = max_steer
        elif alpha < -np.pi/2.0: 
            delta = -max_steer
        else:
            # ref: https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html
            delta = self.normalize_angle(math.atan2(2.0 * WB *  math.sin(alpha), L_d))
        
        # decreasing the desired speed when turning
        if delta > math.radians(10) or delta < -math.radians(10):
            desired_speed = 3
        else:
            desired_speed = 6

        # delta = 3 * delta
        delta = np.clip(delta, -max_steer, max_steer)
        delta = delta
        throttle = 3 * (desired_speed-pose.v)

        return throttle, delta, path, target

    @staticmethod
    def dist(point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)

        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return distance

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

    node = Controller()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()