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
max_acc = json_object["Controller"]["max_acc"]  # [rad] max acceleration
min_acc = json_object["Controller"]["min_acc"]  # [rad] min acceleration
max_speed = json_object["Controller"]["max_speed"]  # [rad] max speed
min_speed = json_object["Controller"]["min_speed"]  # [rad] min speed
controller_type = json_object["Controller"]["controller_type"]
debug = False
robot_num = json_object["robot_num"]
safety = json_object["safety"]
width = json_object["width"]
height = json_object["height"]


class Controller(Node):

    def __init__(self):
        super().__init__("robot_controller")
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
        model_type = self.get_parameter('model_type').get_parameter_value().string_array_value
        
        self.width = width
        self.heigth = height
            
        self.multi_control_pub = self.create_publisher(MultiControl, '/multi_control', 20)
        self.multi_path_pub = self.create_publisher(MultiplePaths, "/robot_multi_traj", 2)
        multi_state_sub = message_filters.Subscriber(self, MultiState, "/robot_multi_state")

        self.controller_type = controller_type

        if self.controller_type == "random_walk":
            ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub], 4, 0.3, allow_headerless=True)
            ts.registerCallback(self.random_walk_controller)
            multi_control = MultiControl()

            for i in range(robot_num):
                multi_control.multi_control.append(ControlInputs(delta=0.0, throttle=0.0))
        
        elif self.controller_type == "random_harem":
            # Initializing the robots
            self.targets = []
            multi_control = MultiControl()

            for i in range(robot_num):
                self.targets.append(random.choice([idx for idx in range(0,robot_num) if idx not in [i]]))
                initial_state = State(x=x0[i], y=y0[i], yaw=yaw[i], v=v[i], omega=omega[i])
                multi_control.multi_control.append(ControlInputs(delta=0.0, throttle=0.0))

            self.time_bkp = time.time()
            ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub], 4, 0.3, allow_headerless=True)
            ts.registerCallback(self.random_harem_callback)  
     
        else:
            # Initializing the robots
            self.paths = []
            self.targets = []
            self.multi_traj = MultiplePaths()
            multi_control = MultiControl()

            for i in range(robot_num):
                self.paths.append(self.create_path())
                self.targets.append([self.paths[i][0].x, self.paths[i][0].y])
                initial_state = State(x=x0[i], y=y0[i], yaw=yaw[i], v=v[i], omega=omega[i])
                self.multi_traj.multiple_path.append(predict_trajectory(initial_state, self.targets[i]))
                # TODO initialize control
                multi_control.multi_control.append(ControlInputs(delta=0.0, throttle=0.0))
            
            ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub], 4, 0.3, allow_headerless=True)
            ts.registerCallback(self.general_pose_callback)
            self.multi_path_pub.publish(self.multi_traj)  
     

        # TODO initialize the initial control in the launch file
        self.multi_control_pub.publish(multi_control)
        # CBF
        self.uni_barrier_cert = create_unicycle_barrier_certificate_with_boundary()
        self.get_logger().info("Controller has been started")

    def random_walk_controller(self, multi_state):

        multi_control = MultiControl()
        for i in range(robot_num):
            cmd = ControlInputs()

            cmd.delta = random.uniform(-max_steer, max_steer)
            if multi_state.multiple_state[i].v >= max_speed:
                cmd.throttle = random.uniform(-max_acc, 0)
            elif multi_state.multiple_state[i].v <= min_speed:
                cmd.throttle = random.uniform(0, max_acc)
            else:
                cmd.throttle = random.uniform(-max_acc, max_acc)
            
            multi_control.multi_control.append(cmd)

        # Applying the CBF
        multi_control = self.apply_CBF(multi_control=multi_control, multi_state=multi_state)

        # Publishing everything in the general callback to avoid deadlocks
        self.multi_control_pub.publish(multi_control)
    
    def random_harem_callback(self, multi_state):
        if time.time() - self.time_bkp > 4:
            self.get_logger().info("Changing the targets")
            self.time_bkp = time.time()
            self.update_targets(multi_state=multi_state)

        multi_control = MultiControl()
        for i in range(robot_num):                                                                                     
            cmd = ControlInputs()
            cmd.throttle, cmd.delta= self.pure_pursuit_steer_control([multi_state.multiple_state[self.targets[i]].x, multi_state.multiple_state[self.targets[i]].y],
                                                                     multi_state.multiple_state[i])
            multi_control.multi_control.append(cmd)
        
        # Applying the CBF
        multi_control = self.apply_CBF(multi_control=multi_control, multi_state=multi_state)

        # Publishing everything in the general callback to avoid deadlocks
        self.multi_control_pub.publish(multi_control)

    def general_pose_callback(self, multi_state):

        multi_control = MultiControl()
        for i in range(robot_num):
            cmd, self.paths[i], self.targets[i], self.multi_traj.multiple_path[i] = self.control_callback(multi_state.multiple_state[i], 
                                                                                                         self.targets[i], self.paths[i], 
                                                                                                         self.multi_traj.multiple_path[i])
            multi_control.multi_control.append(cmd)
        
        # Applying the CBF
        multi_control = self.apply_CBF(multi_control=multi_control, multi_state=multi_state)

        # Publishing everything in the general callback to avoid deadlocks
        self.multi_control_pub.publish(multi_control)
        self.multi_path_pub.publish(self.multi_traj)

    def control_callback(self, pose: FullState, target, path, trajectory):
        # updating target waypoint and predicting new traj
        if self.dist(point1=(pose.x, pose.y), point2=target) < L_d:
            path = self.update_path(path)
            target = (path[0].x, path[0].y)
            trajectory = predict_trajectory(pose, target)
    
        cmd = ControlInputs()
        cmd.throttle, cmd.delta= self.pure_pursuit_steer_control(target, pose)

        if debug:
            self.get_logger().info("Control input robot1, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))

        return cmd, path, target, trajectory
    
    def apply_CBF(self, multi_control: MultiControl, multi_state: MultiState):
        # TODO Remember to change the 3 with the actual number of robots and the two with the input dim not hardcoding
        dxu = np.zeros((2,robot_num))
        x = np.zeros((4,robot_num))

        for i in range(robot_num):
            dxu[0,i], dxu[1,i] = multi_control.multi_control[i].throttle, multi_control.multi_control[i].delta
            x[:,i] = state_to_array(multi_state.multiple_state[i]).reshape(4)
        
        # dxu = CBF(x, dxu)
        dxu = C3BF(x, dxu)

        multi_control = MultiControl()
        for i in range(robot_num):
            multi_control.multi_control.append(ControlInputs(throttle=dxu[0,i], delta=dxu[1,i]))

        return multi_control
    
    def update_targets(self, multi_state: MultiState):
        for i in range(len(self.targets)):
            self.targets[i] = random.choice([idx for idx in range(0,robot_num) if idx not in [i]])
            self.get_logger().info("New targets: " + str(multi_state.multiple_state[self.targets[i]].x) + " " + str(multi_state.multiple_state[self.targets[i]].y))

    def update_path(self, path: Path):
        path.pop(0)
        path.append(Coordinate(x=float(random.randint(-self.width/2, self.width/2)), y=float(random.randint(-self.heigth/2, self.heigth/2))))
        return path

    def create_path(self):
        path = []
        while len(path)<5:
            path.append(Coordinate(x=float(random.randint(-self.width/2, self.width/2)), y=float(random.randint(-self.heigth/2, self.heigth/2))))
        return path
    
    def pure_pursuit_steer_control(self, target, pose: FullState):

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

        return throttle, delta

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