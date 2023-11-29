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

max_steer = json_object["Car_model"]["max_steer"] # [rad] max steering angle
max_speed = json_object["Car_model"]["max_speed"] # [m/s]
min_speed = json_object["Car_model"]["min_speed"] # [m/s]
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
N=2

robot_num = json_object["robot_num"]
timer_freq = json_object["timer_freq"]

show_animation = True

class RobotType(Enum):
    circle = 0
    rectangle = 1

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_acc = 40 # [m/s]
        self.min_acc = -40# [m/s]
        self.max_speed = 10 # [m/s]
        self.min_speed = -1# [m/s]
        self.max_delta = np.radians(45)  # [rad]
        self.max_accel = 0.2  # [m/ss]
        # self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 5# [m/s]
        self.delta_resolution = math.radians(5)  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 50.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.rectangle
        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check
        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([[-1, -1],
                            [0, 4],
                            [5.0, 4.0],
                            [7.0, 5.0],
                            [7.0, 8.0],
                            [9.0, 8.0],
                            [9.0, 11.0],
                            [8.0, 12.0],
                            [9.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 11.0],
                            [12.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]
                            ])

    @property
    def robot_type(self):
        return self._robot_type
    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()

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
    while time <= config.predict_time:
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
    Vs = [config.min_acc, config.max_acc,
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
    print(len(np.arange(dw[0], dw[1], config.v_resolution)))
    print(len(np.arange(dw[2], dw[3], config.delta_resolution)))
    traj = []
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for delta in np.arange(dw[2], dw[3], config.delta_resolution):
            u = np.array([v, delta])
            traj.append(calc_trajectory(x, u, config.dt))

    return traj

def main():
    print(__file__ + " start!!")

    # initial state [x(m), y(m), yaw(rad), v(m/s)]
    x_init = np.array([0.0, 0.0, np.radians(90.0), 0.0])
    traj = generate_trajectories(x_init)

    # for stopping simulation with the esc key.
    # plt.gcf().canvas.mpl_connect(
    #     'key_release_event',
    #     lambda event: [exit(0) if event.key == 'escape' else None])
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
    print(traj.shape)
    line = LineString(zip(traj[0, 0], traj[0,1]))
    dilated = line.buffer(0.5)

    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(121)
    plot_line(line, ax=ax, add_points=False, linewidth=3)
    plot_polygon(dilated, ax=ax, add_points=False, alpha=0.5)
    plt.show()
        
        

if __name__ == '__main__':
    main()