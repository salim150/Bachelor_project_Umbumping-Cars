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

debug = True
WB = 2.9  # [m] Wheel base of vehicle
Lf = 20  # [m] look-ahead distance
kpp = 2

class Controller(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.previous_x = 0
        
        # creating random walk path for car2 to follow
        self.path = []
        self.safety = 20 # safety border around the map boundaries
        self.width = 100 - self.safety
        self.heigth = 100 -self.safety
        self.create_path()
        print(self.path)
        print(self.path[1])
        print(type(self.path))
        self.target = (self.path[0].x, self.path[0].y)
    
        self.control1_publisher_ = self.create_publisher(ControlInputs, "/robot1_control", 20)
        self.control2_publisher_ = self.create_publisher(ControlInputs, "/robot2_control", 20)
        self.path_pub = self.create_publisher(Path, "/robot2_path", 2)
        
        state1_subscriber = message_filters.Subscriber(self, State, "/robot1_measurement")
        state2_subscriber = message_filters.Subscriber(self, State, "/robot2_measurement")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber], 2, 0.1, allow_headerless=True)
        ts.registerCallback(self.general_pose_callback)

        self.control1_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))
        self.control2_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))
        self.path_pub.publish(Path(path=self.path))

        self.get_logger().info("Controller has been started")

    def general_pose_callback(self, state1: State, state2: State):
        
        self.pose1_callback(state1)
        self.pose2_callback(state2)
        
        
    def pose1_callback(self, pose: State):
    
        cmd = ControlInputs()
        cmd.delta =15.0
        cmd.throttle = 3.0
        self.control1_publisher_.publish(cmd)
        
        if debug:
            self.get_logger().info("Control input robot1, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))

    def pose2_callback(self, pose: State):
    
        cmd = ControlInputs()
        # updating target waypoint
        if self.dist(point1=(pose.x, pose.y), point2=self.target) < Lf:
            self.update_path()
            self.target = (self.path[0].x, self.path[0].y)

        alpha = self.normalize_angle(math.atan2(self.target[1] - pose.y, self.target[0] - pose.x) - pose.yaw)

        # this if/else condition should fix the buf of the waypoint behind the car
        if alpha > np.pi/2.0:
            cmd.delta = math.radians(15)
        elif alpha < -np.pi/2.0: 
            cmd.delta = math.radians(-15)
        else:
            # ref: https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html
            cmd.delta = self.normalize_angle(math.atan2(2.0 * WB *  math.sin(alpha), Lf))
        
        # decreasing the desired speed when turning
        if cmd.delta > math.radians(10) or cmd.delta < -math.radians(10):
            desired_speed = 3
        else:
            desired_speed = 6

        cmd.delta = math.degrees(cmd.delta)
        cmd.throttle = 3 * (desired_speed-pose.v)

        """cmd.delta = 15.0
        cmd.throttle = 3.0"""
        self.control2_publisher_.publish(cmd)
        self.path_pub.publish(Path(path=self.path))
        
        if debug:
            self.get_logger().info("Control input robot2, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))
    
    def update_path(self):
        self.path.pop(0)
        self.path.append(Coordinate(x=random.randint(-self.width/2, self.width/2), y=random.randint(-self.heigth/2, self.heigth/2)))

    def create_path(self):
        while len(self.path)<5:
            self.path.append(Coordinate(x=random.randint(-self.width/2, self.width/2), y=random.randint(-self.heigth/2, self.heigth/2)))
    
    def pure_pursuit_steer_control(self):
        if self.dist(point1=(self._current_x, self._current_y), point2=self.target) < Lf and self.idx < len(
                self._waypoints) - 1:
            self._waypoints.pop(self.idx)
            self.target = self._waypoints[self.idx]

        if self.idx < len(self._waypoints) and self.dist((self._current_x, self._current_y), self._waypoints[-1]) < 10:
            self._desired_speed = 0
            self.steer = 0

        alpha = math.atan2(self.target[1] - self._current_y, self.target[0] - self._current_x) - self._current_yaw

        self.steer = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

        # decreasing the desired speed when turning
        if self.steer > math.radians(10) or self.steer < -math.radians(10):
            self._desired_speed = 5
        else:
            self._desired_speed = 9

        self.throttle = self.proportional_control()

    def proportional_control(self):
        a = 3 * (self._desired_speed-self._current_speed)
        return a

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