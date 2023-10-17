#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State, Path, Coordinate
from std_msgs.msg import Float32MultiArray
import message_filters
import random
import math
import numpy as np
from planner.cubic_spline_planner import *
from planner.frenet import *
from planner.predict_traj import *

# for the CBF
from planner.CBF import *

debug = False
WB = 2.9  # [m] Wheel base of vehicle
Lf = 20  # [m] look-ahead distance
max_steer = np.radians(30.0)  # [rad] max steering angle

class Controller(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.previous_x = 0
        
        # creating random walk path for car2 to follow
        self.safety = 20 # safety border around the map boundaries
        self.width = 100.0 - self.safety
        self.heigth = 100.0 -self.safety
        print(type(self.width))

        self.path1 = []
        self.path1 = self.create_path(self.path1)
        self.target1 = (self.path1[0].x, self.path1[0].y)
        self.trajectory1, self.tx1, self.ty1 = predict_trajectory(State(x=30.0, y=30.0, yaw=0.0, v=0.0, omega=0.0), self.target1)

        self.path2 = []
        self.path2 = self.create_path(self.path2)
        self.target2 = (self.path2[0].x, self.path2[0].y)
        self.trajectory2, self.tx2, self.ty2 = predict_trajectory(State(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0), self.target2)

        # Just to try if frenet path works
        # initial state
        self.c_speed = 10.0 / 3.6  # current speed [m/s]
        self.c_accel = 0.0  # current acceleration [m/ss]
        self.c_d = 2.0  # current lateral position [m]
        self.c_d_d = 0.0  # current lateral speed [m/s]
        self.c_d_dd = 0.0  # current lateral acceleration [m/s]
        self.s0 = 0.0  # current course position
        self.area = 50.0  # animation area length [m]
    
        self.control1_publisher_ = self.create_publisher(ControlInputs, "/robot1_control", 20)
        self.control2_publisher_ = self.create_publisher(ControlInputs, "/robot2_control", 20)
        self.path_pub = self.create_publisher(Path, "/robot2_path", 2)
        self.trajectory_pub1 = self.create_publisher(Path, "/robot1_trajectory", 2)
        self.trajectory_pub2 = self.create_publisher(Path, "/robot2_trajectory", 2)
        
        state1_subscriber = message_filters.Subscriber(self, State, "/robot1_measurement")
        state2_subscriber = message_filters.Subscriber(self, State, "/robot2_measurement")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber], 2, 0.1, allow_headerless=True)
        ts.registerCallback(self.general_pose_callback)

        self.control1_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))
        self.control2_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))
        self.path_pub.publish(Path(path=self.path2))

        # CBF
        self.uni_barrier_cert = create_unicycle_barrier_certificate_with_boundary()

        self.get_logger().info("Controller has been started")

    def general_pose_callback(self, state1: State, state2: State):
        
        cmd1 = self.pose1_callback(state1)
        cmd2 = self.pose2_callback(state2, state1)

        self.get_logger().info(f'Commands before: cmd1: {cmd1}, cmd2: {cmd2}')
        dxu = np.zeros((2,2))
        dxu[0,0], dxu[1,0] = cmd1.throttle, cmd1.delta
        dxu[0,1], dxu[1,1] = cmd2.throttle, cmd2.delta

        # Converting positions to arrays for the CBF
        x1 = state_to_array(state1)
        x2 = state_to_array(state2)
        x = np.concatenate((x1, x2), axis=1)

        # Create safe control inputs (i.e., no collisions)
        dxu = self.uni_barrier_cert(dxu, x)

        cmd1.throttle, cmd1.delta = dxu[0,0], dxu[1,0]
        cmd2.throttle, cmd2.delta = dxu[0,1], dxu[1,1]

        self.control1_publisher_.publish(cmd1)
        self.control2_publisher_.publish(cmd2)
        self.get_logger().info(f'Commands after: cmd1: {cmd1}, cmd2: {cmd2}')

    def pose1_callback(self, pose: State):
        # updating target waypoint and predicting new traj
        if self.dist(point1=(pose.x, pose.y), point2=self.target1) < Lf:
            self.path1 = self.update_path(self.path1)
            self.target1 = (self.path1[0].x, self.path1[0].y)
            self.trajectory1, self.tx1, self.ty1  = predict_trajectory(pose, self.target1)
    
        cmd = ControlInputs()
        cmd.throttle, cmd.delta, self.path1, self.target1 = self.pure_pursuit_steer_control(self.target1, pose, self.path1)
        self.trajectory_pub1.publish(Path(path=self.trajectory1))
        # self.control1_publisher_.publish(cmd)
        
        if debug:
            self.get_logger().info("Control input robot1, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))

        return cmd
    
    def pose2_callback(self, pose: State, other_pose: State):       

    # updating target waypoint and predicting new traj
        if self.dist(point1=(pose.x, pose.y), point2=self.target2) < Lf:
            self.path2 = self.update_path(self.path2)
            self.target2 = (self.path2[0].x, self.path2[0].y)
            self.trajectory2, self.tx2, self.ty2  = predict_trajectory(pose, self.target2)

        cmd = ControlInputs()       
        cmd.throttle, cmd.delta, self.path2, self.target2 = self.pure_pursuit_steer_control(self.target2, pose, self.path2)
        self.path_pub.publish(Path(path=self.path2))
        self.trajectory_pub2.publish(Path(path=self.trajectory2))
        
        if debug:
            self.get_logger().info("Control input robot2, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))
        
        return cmd
    
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
    
    def pure_pursuit_steer_control(self, target, pose: Pose, path: Path):

        alpha = self.normalize_angle(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)

        # this if/else condition should fix the buf of the waypoint behind the car
        if alpha > np.pi/2.0:
            delta = max_steer
        elif alpha < -np.pi/2.0: 
            delta = -max_steer
        else:
            # ref: https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html
            delta = self.normalize_angle(math.atan2(2.0 * WB *  math.sin(alpha), Lf))
        
        # decreasing the desired speed when turning
        if delta > math.radians(10) or delta < -math.radians(10):
            desired_speed = 3
        else:
            desired_speed = 6

        delta = 3 * delta
        delta = np.clip(delta, -max_steer, max_steer)
        delta = math.degrees(delta)
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