#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_message.msg import MultiState, FullState, State
import message_filters
import time
from geometry_msgs.msg import Pose, PoseArray
import math
import numpy as np

# For the parameter file
import pathlib
import json
from geometry_msgs.msg import Pose

path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

debug = False
robot_num = json_object["robot_num"]
timer_freq = json_object["timer_freq"]

class Converter(Node):

    def __init__(self):
        super().__init__("converter")

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

        x0 = self.get_parameter('x0').get_parameter_value().double_array_value
        y0 = self.get_parameter('y0').get_parameter_value().double_array_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_array_value
        v = self.get_parameter('v').get_parameter_value().double_array_value
        omega = self.get_parameter('omega').get_parameter_value().double_array_value
        # model_type = self.get_parameter('model_type').get_parameter_value().string_array_value

        self.pose_array_state = PoseArray()
        self.pose_array_state.header.frame_id = "base_link"
        self.pose_array_steering = PoseArray()
        self.pose_array_steering.header.frame_id = "base_link"

        # Initializing the robots TODO add in the launch file parameters the initial control--> here it's hardcoded
        for i in range(robot_num):
            pose = self.convert_to_pose(FullState(x=x0[i], y=y0[i], yaw=yaw[i], v=v[i],
                                                             omega=omega[i], delta=0.0, throttle=0.0))
            steering = self.convert_to_steering_pose(FullState(x=x0[i], y=y0[i], yaw=yaw[i], v=v[i], omega=omega[i],
                                                             delta=0.0, throttle=0.0))
            self.pose_array_state.poses.append(pose)
            self.pose_array_steering.poses.append(steering)
        
        self.pose_array_pub = self.create_publisher(PoseArray, "/pose_array", 2)
        self.pose_array_steering_pub = self.create_publisher(PoseArray, "/pose_array_steering", 2)
        self.timer = self.create_timer(timer_freq, self.timer_callback)

        multi_state_subscriber = message_filters.Subscriber(self, MultiState, "/multi_fullstate")
        ts = message_filters.ApproximateTimeSynchronizer([multi_state_subscriber], 6, 1, allow_headerless=True)
        ts.registerCallback(self.state_converter_callback)

        self.get_logger().info("Converter has been started")

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
        
    def convert_to_pose(self, state: FullState):
        pose = Pose()
        pose.position.x = state.x
        pose.position.y = state.y
        pose.position.z = 0.0

        q = self.get_quaternion_from_euler(0, 0, state.yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose
    
    def convert_to_steering_pose(self, state: FullState):
        pose = Pose()
        pose.position.x = state.x
        pose.position.y = state.y
        pose.position.z = 0.0

        q = self.get_quaternion_from_euler(0, 0, state.yaw+state.delta)
        # q = self.quaternion_from_euler(0, 0, state.yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def state_converter_callback(self, multi_state_in: MultiState):
        self.pose_array_state = PoseArray()
        self.pose_array_state.header.frame_id = "base_link"

        self.pose_array_steering = PoseArray()
        self.pose_array_steering.header.frame_id = "base_link"

        for state in multi_state_in.multiple_state:
            pose = self.convert_to_pose(state)
            steering = self.convert_to_steering_pose(state)
            self.pose_array_steering.poses.append(steering)
            self.pose_array_state.poses.append(pose)
        
    def timer_callback(self):
        self.pose_array_pub.publish(self.pose_array_state)
        self.pose_array_steering_pub.publish(self.pose_array_steering)


def main(args=None):
    rclpy.init(args=args)

    node = Converter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()