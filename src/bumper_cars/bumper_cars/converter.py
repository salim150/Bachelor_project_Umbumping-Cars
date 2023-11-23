#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_message.msg import MultiState, FullState, State
import message_filters
import time
from geometry_msgs.msg import Pose, PoseArray
import math

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

        self.pose_array = PoseArray()

        # Initializing the robots TODO add in the launch file parameters the initial control--> here it's hardcoded
        for i in range(robot_num):
            pose = self.convert_to_pose(FullState(x=x0[i], y=y0[i], yaw=yaw[i], v=v[i],
                                                             omega=omega[i], delta=0.0, throttle=0.0))
            self.pose_array.poses.append(pose)
        
        self.pose_array_pub = self.create_publisher(PoseArray, "/pose_array", 2)
        self.timer = self.create_timer(timer_freq, self.timer_callback)

        multi_state_subscriber = message_filters.Subscriber(self, MultiState, "/multi_fullstate")
        ts = message_filters.ApproximateTimeSynchronizer([multi_state_subscriber], 6, 1, allow_headerless=True)
        ts.registerCallback(self.converter_callback)

        self.get_logger().info("Converter has been started")

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
        
    def convert_to_pose(self, state: FullState):
        pose = Pose()
        pose.position.x = state.x
        pose.position.y = state.y
        pose.position.z = 0.0

        q = self.quaternion_from_euler(0, 0, state.yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def converter_callback(self, multi_state_in: MultiState):
        self.pose_array = PoseArray()
        for state in multi_state_in.multiple_state:
            pose = self.convert_to_pose(state)
            self.pose_array.poses.append(pose)
        
    def timer_callback(self):
        self.pose_array_pub.publish(self.pose_array)


def main(args=None):
    rclpy.init(args=args)

    node = Converter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()