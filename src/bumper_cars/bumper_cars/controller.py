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
        
        # Initializing the robots
        initial_state1 = State(x=x0[0], y=y0[0], yaw=yaw[0], v=v[0], omega=omega[0])
        initial_state2 = State(x=x0[1], y=y0[1], yaw=yaw[1], v=v[1], omega=omega[1])
        initial_state3 = State(x=x0[2], y=y0[2], yaw=yaw[2], v=v[2], omega=omega[2])
        
        # creating random walk path for car2 to follow
        self.safety = 20 # safety border around the map boundaries
        self.width = 100.0 - self.safety
        self.heigth = 100.0 -self.safety

        # Robot 1
        self.path1 = []
        self.path1 = self.create_path(self.path1)
        self.target1 = (self.path1[0].x, self.path1[0].y)
        self.trajectory1, self.tx1, self.ty1 = predict_trajectory(initial_state1, self.target1)

        # Robot 2
        self.path2 = []
        self.path2 = self.create_path(self.path2)
        self.target2 = (self.path2[0].x, self.path2[0].y)
        self.trajectory2, self.tx2, self.ty2 = predict_trajectory(initial_state2, self.target2)

        # Robot 3
        self.path3 = []
        self.path3 = self.create_path(self.path3)
        self.target3 = (self.path3[0].x, self.path3[0].y)
        self.trajectory3, self.tx3, self.ty3 = predict_trajectory(initial_state3, self.target3)

        self.multi_control_pub = self.create_publisher(MultiControl, '/multi_control', 20)
        self.multi_path_pub = self.create_publisher(MultiplePaths, "/robot_multi_traj", 2)
        multi_state_sub = message_filters.Subscriber(self, MultiState, "/robot_multi_state")

        # TODO: Pass it from parameters file
        self.controller_type = controller_type

        if self.controller_type == "random_walk":
            ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub], 4, 0.3, allow_headerless=True)
            ts.registerCallback(self.random_walk_controller)
        else:
            ts = message_filters.ApproximateTimeSynchronizer([multi_state_sub], 4, 0.3, allow_headerless=True)
            ts.registerCallback(self.general_pose_callback)

        self.multi_control_pub.publish(MultiControl(multi_control=[ControlInputs(delta=0.0, throttle=0.0), ControlInputs(delta=0.0, throttle=0.0), 
                                                                   ControlInputs(delta=0.0, throttle=0.0)]))
        multi_path = MultiplePaths(multiple_path=[Path(path=self.trajectory1), Path(path=self.trajectory2), Path(path=self.trajectory3)])
        self.multi_path_pub.publish(multi_path)  
     
        # CBF
        self.uni_barrier_cert = create_unicycle_barrier_certificate_with_boundary()
        self.get_logger().info("Controller has been started")

    def random_walk_controller(self, multi_state):

        cmd1 = ControlInputs()
        cmd2 = ControlInputs()
        cmd3 = ControlInputs()

        state1 = multi_state.multiple_state[0]
        state2 = multi_state.multiple_state[1]
        state3 = multi_state.multiple_state[2]

        # Commands for robot 1
        cmd1.delta = random.uniform(-max_steer, max_steer)
        if state1.v >= max_speed:
            cmd1.throttle = random.uniform(-max_acc, 0)
        elif state1.v <= min_speed:
            cmd1.throttle = random.uniform(0, max_acc)
        else:
            cmd1.throttle = random.uniform(-max_acc, max_acc)
        
        # Commands for robot 2
        cmd2.delta = random.uniform(-max_steer, max_steer)
        if state2.v >= max_speed:
            cmd2.throttle = random.uniform(-max_acc, 0)
        elif state2.v <= min_speed:
            cmd2.throttle = random.uniform(0, max_acc)
        else:
            cmd2.throttle = random.uniform(-max_acc, max_acc)

        # Commands for robot 3
        cmd3.delta = random.uniform(-max_steer, max_steer)
        if state3.v >= max_speed:
            cmd3.throttle = random.uniform(-max_acc, 0)
        elif state3.v <= min_speed:
            cmd3.throttle = random.uniform(0, max_acc)
        else:
            cmd3.throttle = random.uniform(-max_acc, max_acc)
        
        # Applying the CBF
        dxu = self.apply_CBF(cmd1=cmd1, cmd2=cmd2, cmd3=cmd3, 
                             state1=state1, state2=state2, state3=state3)

        cmd1.throttle, cmd1.delta = dxu[0,0], dxu[1,0]
        cmd2.throttle, cmd2.delta = dxu[0,1], dxu[1,1]
        cmd3.throttle, cmd3.delta = dxu[0,2], dxu[1,2]

        # Publishing everything in the general callback to avoid deadlocks
        multi_control = MultiControl(multi_control=[cmd1, cmd2, cmd3])
        self.multi_control_pub.publish(multi_control)
    
    def general_pose_callback(self, multi_state):
        
        # debug_time = time.time()

        state1 = multi_state.multiple_state[0]
        state2 = multi_state.multiple_state[1]
        state3 = multi_state.multiple_state[2]

        cmd1, self.path1, self.target1, self.trajectory1 = self.control_callback(state1, self.target1, self.path1, self.trajectory1)
        cmd2, self.path2, self.target2, self.trajectory2 = self.control_callback(state2, self.target2, self.path2, self.trajectory2)
        cmd3, self.path3, self.target3, self.trajectory3 = self.control_callback(state3, self.target3, self.path3, self.trajectory3)

        if debug:
           self.get_logger().info(f'Commands before: cmd1: {cmd1}, cmd2: {cmd2}')
        
        # Applying the CBF
        dxu = self.apply_CBF(cmd1=cmd1, cmd2=cmd2, cmd3=cmd3, 
                             state1=state1, state2=state2, state3=state3)

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

        # print(time.time()-debug_time)
        # debug_time = time.time()

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
    
    def apply_CBF(self, cmd1, cmd2, cmd3, state1, state2, state3):
        # Remember to change the 3 with the actual number of robots
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
        # dxu = self.uni_barrier_cert(dxu, x)
        # dxu = CBF(x, dxu)
        dxu = C3BF(x, dxu)

        return dxu

    def update_path(self, path: Path):
        path.pop(0)
        path.append(Coordinate(x=float(random.randint(-self.width/2, self.width/2)), y=float(random.randint(-self.heigth/2, self.heigth/2))))
        return path

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