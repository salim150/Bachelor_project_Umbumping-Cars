#!/usr/bin/env python3

from custom_message.msg import ControlInputs, State
import numpy as np
from planner.cubic_spline_planner import *
from planner.frenet import *
from planner.predict_traj import *

# TODO: import all this parameters from a config file so that we can easily change them in one place
L = 2.9
max_steer = np.radians(30.0)  # [rad] max steering angle
max_speed = 6 # [m/s]
min_speed = 0.3 # [m/s]
magnitude_limit= max_speed
max_acc = 40
min_acc = -40
dt = 0.1
safety_radius = 3
barrier_gain = 100
magnitude_limit = max_speed
Kv = 1
L = 2.9  # [m] Wheel base of vehicle
Lr = L / 2.0  # [m]
Lf = L - Lr

def linear_model_callback(initial_state: State, cmd: ControlInputs):

    dt = 0.1
    state = State()
    cmd.delta = np.clip(np.radians(cmd.delta), -max_steer, max_steer)

    state.x = initial_state.x + initial_state.v * np.cos(initial_state.yaw) * dt
    state.y = initial_state.y + initial_state.v * np.sin(initial_state.yaw) * dt
    state.yaw = initial_state.yaw + initial_state.v / L * np.tan(cmd.delta) * dt
    state.yaw = normalize_angle(state.yaw)
    state.v = initial_state.v + cmd.throttle * dt
    state.v = np.clip(state.v, min_speed, max_speed)

    return state

def nonlinear_model_callback(initial_state: State, cmd: ControlInputs, old_time: float):

    dt = 0.1
    state = State()
    #dt = time.time() - old_time
    cmd.delta = np.clip(np.radians(cmd.delta), -max_steer, max_steer)

    beta = math.atan2((Lr * math.tan(cmd.delta) / L), 1.0)
    vx = initial_state.v * math.cos(beta)
    vy = initial_state.v * math.sin(beta)

    Ffy = -Cf * ((vy + Lf * initial_state.omega) / (vx + 0.0001) - cmd.delta)
    Fry = -Cr * (vy - Lr * initial_state.omega) / (vx + 0.0001)
    R_x = c_r1 * abs(vx)
    F_aero = c_a * vx ** 2 # 
    F_load = F_aero + R_x #
    state.omega = initial_state.omega + (Ffy * Lf * math.cos(cmd.delta) - Fry * Lr) / Iz * dt
    vx = vx + (cmd.throttle - Ffy * math.sin(cmd.delta) / m - F_load / m + vy * state.omega) * dt
    vy = vy + (Fry / m + Ffy * math.cos(cmd.delta) / m - vx * state.omega) * dt

    state.yaw = initial_state.yaw + state.omega * dt
    state.yaw = normalize_angle(state.yaw)

    state.x = initial_state.x + vx * math.cos(state.yaw) * dt - vy * math.sin(state.yaw) * dt
    state.y = initial_state.y + vx * math.sin(state.yaw) * dt + vy * math.cos(state.yaw) * dt

    state.v = math.sqrt(vx ** 2 + vy ** 2)
    state.v = np.clip(state.v, min_speed, max_speed)

    return state, time.time()

def pure_pursuit_steer_control(target, pose):
        
    alpha = normalize_angle(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)

    # this if/else condition should fix the buf of the waypoint behind the car
    if alpha > np.pi/2.0:
        delta = max_steer
    elif alpha < -np.pi/2.0:
        delta = -max_steer
    else:
        # ref: https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html
        delta = normalize_angle(math.atan2(2.0 * WB *  math.sin(alpha), Lf))

    # decreasing the desired speed when turning
    if delta > math.radians(10) or delta < -math.radians(10):
        desired_speed = 3
    else:
        desired_speed = 6

    delta = np.clip(delta, -max_steer, max_steer)
    delta = math.degrees(delta)
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
    return float(distance)

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

def array_to_state(array):
    state = State()
    state.x = array[0]
    state.y = array[1]
    state.yaw = array[2]
    state.v = array[3]
    return state

def state_to_array(state: State):
    array = np.zeros((4,1))
    array[0,0] = state.x
    array[1,0] = state.y
    array[2,0] = state.yaw
    array[3,0] = state.v

    return array

def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        
def plot_map(width=100, heigth=100):
        corner_x = [-width/2.0, width/2.0, width/2.0, -width/2.0, -width/2.0]
        corner_y = [heigth/2.0, heigth/2.0, -heigth/2.0, -heigth/2.0, heigth/2.0]

        plt.plot(corner_x, corner_y)

def plot_path(path: Path):
        x = []
        y = []
        for coord in path:
            x.append(coord.x)
            y.append(coord.y)
        plt.scatter(x, y, marker='.', s=10)
        plt.scatter(x[0], y[0], marker='x', s=20)

def normalize_angle_array(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    angle[angle[:] > np.pi] -= 2.0 * np.pi

    angle[angle[:] < -np.pi] += 2.0 * np.pi

    return angle