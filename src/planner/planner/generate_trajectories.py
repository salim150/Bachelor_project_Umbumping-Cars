import matplotlib.pyplot as plt
import numpy as np
import math
from enum import Enum
# For the parameter file
import pathlib
import json
from custom_message.msg import ControlInputs
from shapely.geometry import Point, Polygon, LineString
from shapely.plotting import plot_polygon, plot_line

path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

max_steer = json_object["DWA"]["max_steer"] # [rad] max steering angle
max_speed = json_object["DWA"]["max_speed"] # [m/s]
min_speed = json_object["DWA"]["min_speed"] # [m/s]
v_resolution = json_object["DWA"]["v_resolution"] # [m/s]
delta_resolution = math.radians(json_object["DWA"]["delta_resolution"])# [rad/s]
max_acc = json_object["DWA"]["max_acc"] # [m/ss]
min_acc = json_object["DWA"]["min_acc"] # [m/ss]
dt = json_object["DWA"]["dt"] # [s] Time tick for motion prediction
predict_time = json_object["DWA"]["predict_time"] # [s]
to_goal_cost_gain = json_object["DWA"]["to_goal_cost_gain"]
speed_cost_gain = json_object["DWA"]["speed_cost_gain"]
obstacle_cost_gain = json_object["DWA"]["obstacle_cost_gain"]
robot_stuck_flag_cons = json_object["DWA"]["robot_stuck_flag_cons"]
dilation_factor = json_object["DWA"]["dilation_factor"]
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
robot_num = json_object["robot_num"]
safety_init = json_object["safety"]
width_init = json_object["width"]
height_init = json_object["height"]
N=3

robot_num = json_object["robot_num"]
timer_freq = json_object["timer_freq"]

show_animation = True

class RobotType(Enum):
    circle = 0
    rectangle = 1

robot_type = RobotType.rectangle
robot_radius = 1.0

def motion(x, u, dt):
    """
    motion model
    initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    """
    delta = u[1]
    delta = np.clip(delta, -max_steer, max_steer)
    throttle = u[0]

    x[0] = x[0] + x[3] * math.cos(x[2]) * dt
    x[1] = x[1] + x[3] * math.sin(x[2]) * dt
    x[2] = x[2] + x[3] / L * math.tan(delta) * dt
    x[3] = x[3] + throttle * dt
    x[2] = normalize_angle(x[2])
    x[3] = np.clip(x[3], min_speed, max_speed)

    return x

def normalize_angle(angle):
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


def calc_trajectory(x_init, u, dt):
    """
    calc trajectory
    """
    x = np.array(x_init)
    traj = np.array(x)
    time = 0.0
    while time <= predict_time:
        x = motion(x, u, dt)
        traj = np.vstack((traj, x))
        time += dt
    return traj

def calc_dynamic_window(x):
    """
    calculation dynamic window based on current state x
    motion model
    initial state [x(m), y(m), yaw(rad), v(m/s), delta(rad)]
    """
    # Dynamic window from robot specification
    Vs = [min_acc, max_acc,
          -max_steer, max_steer]
    
    
    # Dynamic window from motion model
    # Vd = [x[3] - config.max_acc*0.1,
    #       x[3] + config.max_acc*0.1,
    #       -max_steer,
    #       max_steer]
    
    # #  [min_throttle, max_throttle, min_steer, max_steer]
    # dw = [min(Vs[0], Vd[0]), min(Vs[1], Vd[1]), min(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    dw = [Vs[0], Vs[1], Vs[2], Vs[3]]
    
    return dw

def generate_trajectories(x):
    """
    Generate trajectories
    """
    dw = calc_dynamic_window(x)
    print(np.arange(dw[0], dw[1]+v_resolution, v_resolution))
    print(np.arange(dw[2], dw[3]+delta_resolution, delta_resolution))
    traj = []
    for v in np.arange(dw[0], dw[1]+v_resolution, v_resolution):
        for delta in np.arange(dw[2], dw[3]+delta_resolution, delta_resolution):
            u = np.array([v, delta])
            traj.append(calc_trajectory(x, u, dt))

    return traj

def main():
    print(__file__ + " start!!")

    # initial state [x(m), y(m), yaw(rad), v(m/s)]
    x_init = np.array([0.0, 0.0, np.radians(90.0), 0.0])
    traj = generate_trajectories(x_init)

    plt.plot(x_init[0], x_init[1], "xr")
    for trajectory in traj:
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-g")
        # plt.plot(x_goal[0], x_goal[1], "xb")
        plt.grid(True)
        plt.axis("equal")

    plt.show()

    traj = np.array(traj)

    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(121)

    for i in range(len(traj)):
        line = LineString(zip(traj[i, :, 0], traj[i, :, 1]))
        dilated = line.buffer(0.5, cap_style=3, join_style=3)
        plot_line(line, ax=ax, add_points=False, linewidth=3)
        plot_polygon(dilated, ax=ax, add_points=False, alpha=0.5)

    plt.show()
        
        

if __name__ == '__main__':
    main()