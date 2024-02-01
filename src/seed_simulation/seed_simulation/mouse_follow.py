# from pynput.mouse import Listener
import matplotlib.pyplot as plt
import numpy as np
import math
from enum import Enum
# For the parameter file
import pathlib
import json
from custom_message.msg import Coordinate
from shapely.geometry import Point, Polygon, LineString
from shapely import intersection, distance
from shapely.plotting import plot_polygon, plot_line
import planner.utils as utils
from lbp_dev.LBP import *
# for debugging
import time
from matplotlib.animation import FuncAnimation
from numpy import cos, sin

path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

max_steer = json_object["LBP"]["max_steer"] # [rad] max steering angle
max_speed = json_object["LBP"]["max_speed"] # [m/s]
min_speed = json_object["LBP"]["min_speed"] # [m/s]
v_resolution = json_object["LBP"]["v_resolution"] # [m/s]
delta_resolution = math.radians(json_object["LBP"]["delta_resolution"])# [rad/s]
max_acc = 10 #json_object["LBP"]["max_acc"] # [m/ss]
min_acc = -10 #json_object["LBP"]["min_acc"] # [m/ss]
dt = json_object["LBP"]["dt"] # [s] Time tick for motion prediction
predict_time = json_object["LBP"]["predict_time"] # [s]
to_goal_cost_gain = json_object["LBP"]["to_goal_cost_gain"]
speed_cost_gain = json_object["LBP"]["speed_cost_gain"]
obstacle_cost_gain = json_object["LBP"]["obstacle_cost_gain"]
heading_cost_gain = json_object["LBP"]["heading_cost_gain"]
robot_stuck_flag_cons = json_object["LBP"]["robot_stuck_flag_cons"]
dilation_factor = json_object["LBP"]["dilation_factor"]

L = json_object["Car_model"]["L"]  # [m] Wheel base of vehicle
Lr = L / 2.0  # [m]
Lf = L - Lr
Cf = json_object["Car_model"]["Cf"]  # N/rad
Cr = json_object["Car_model"]["Cr"] # N/rad
Iz = json_object["Car_model"]["Iz"]  # kg/m2
m = json_object["Car_model"]["m"]  # kg
# Aerodynamic and friction coefficients
c_a = json_object["Car_model"]["c_a"]
c_r1 = json_object["Car_model"]["c_r1"]
WB = json_object["Controller"]["WB"] # Wheel base
L_d = json_object["Controller"]["L_d"]  # [m] look-ahead distance
robot_num = json_object["robot_num"]
safety_init = json_object["safety"]
width_init = json_object["width"]
height_init = json_object["height"]
min_dist = json_object["min_dist"]
update_dist = 2
N=3

show_animation = True
check_collision_bool = False
color_dict = {0: 'r', 1: 'b', 2: 'g', 3: 'y', 4: 'm', 5: 'c', 6: 'k'}

# Simple mouse click function to store coordinates
def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata

    # assign global variable to access outside of function
    global coords
    coords.pop(0)
    coords.append((ix, iy))
    print(coords[-1])

    return

coords = [(0,0)]
fig = plt.figure(1, dpi=90, figsize=(10,10))
ax = fig.add_subplot(111)

cid = fig.canvas.mpl_connect('button_press_event', onclick)

def main_seed():
    """
    This function runs the main loop for the LBP algorithm.
    It initializes the necessary variables, updates the robot state, and plots the robot trajectory.

    The simulation if this function has a variable amout of robots robot_num defined in the parameter file.
    THis is the core a reference implementation of the LBP algorithm with random generation of goals that are updated when 
    the robot reaches the current goal.
    """
    print(__file__ + " start!!")
    iterations = 3000
    break_flag = False

    # Step 2: Sample initial values for x0, y, yaw, v, omega, and model_type
    initial_state = seed['initial_position']
    x0 = initial_state['x']
    y = initial_state['y']
    yaw = initial_state['yaw']
    v = initial_state['v']

    # Step 3: Create an array x with the initial values
    x = np.array([x0, y, yaw, v])
    u = np.zeros((2, robot_num))

    trajectory = np.zeros((x.shape[0], robot_num, 1))
    trajectory[:, :, 0] = x

    predicted_trajectory = dict.fromkeys(range(robot_num),np.zeros([int(predict_time/dt), 3]))
    for i in range(robot_num):
        predicted_trajectory[i] = np.full((int(predict_time/dt), 3), x[0:3,i])

    # Step 4: Create paths for each robot
    traj = seed['trajectories']
    paths = [[Coordinate(x=traj[str(idx)][i][0], y=traj[str(idx)][i][1]) for i in range(len(traj[str(idx)]))] for idx in range(robot_num)]

    # Step 5: Extract the target coordinates from the paths
    targets = [[path[0].x, path[0].y] for path in paths]

    # Step 6: Create dilated trajectories for each robot
    dilated_traj = []

    global coords
    coords = [(10,10)]
    for i in range(robot_num):
        dilated_traj.append(Point(x[0, i], x[1, i]).buffer(dilation_factor, cap_style=3))

    u_hist = dict.fromkeys(range(robot_num),[0]*int(predict_time/dt))
    # fig = plt.figure(1, dpi=90)
    # ax = fig.add_subplot(111)
    
    lbp = LBP_algorithm(x, predicted_trajectory, robot_num, safety_init, 
                        width_init, height_init, min_dist, paths, targets, dilated_traj,
                        predicted_trajectory, ax, u_hist)
    
    for z in range(iterations):
        plt.cla()
        plt.xlim(-width_init, width_init)
        plt.ylim(-height_init, height_init)
        
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        
        x, u, break_flag = lbp.run_lbp(x, u, break_flag)
        # piloting the first robot
        lbp.targets[0] = (coords[-1][0], coords[-1][1])

        trajectory = np.dstack([trajectory, x])

        utils.plot_map(width=width_init, height=height_init)
        plt.plot(coords[-1][0], coords[-1][1], 'k', marker='o', markersize=20)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

        if break_flag:
            break

    print("Done")
    if show_animation:
        for i in range(robot_num):
            plt.plot(trajectory[0, i, :], trajectory[1, i, :], "-r")
        plt.pause(0.0001)
        plt.show()

if __name__ == '__main__':
    main_seed()