#!/usr/bin/env python3

from custom_message.msg import ControlInputs, State
import numpy as np
from planner.cubic_spline_planner import *
from planner.frenet import *
from planner.predict_traj import *
from scipy.spatial.transform import Rotation as Rot

# For the parameter file
import pathlib
import json

# TODO: import all this parameters from a config file so that we can easily change them in one place
path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

max_steer = json_object["CBF_simple"]["max_steer"]   # [rad] max steering angle
max_speed = json_object["CBF_simple"]["max_speed"] # [m/s]
min_speed = json_object["CBF_simple"]["min_speed"]  # [m/s]
dt = json_object["CBF_simple"]["dt"] 
L = json_object["CBF_simple"]["L"] # [m] Wheel base of vehicle
Lr = L / 2.0  # [m]
Lf = L - Lr

def linear_model_callback(initial_state: State, cmd: ControlInputs):

    dt = 0.1
    state = State()
    cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)
    cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)

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
    cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)
    cmd.delta = np.clip(cmd.delta, -max_steer, max_steer)

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
        desired_speed = max_speed

    delta = np.clip(delta, -max_steer, max_steer)
    # delta = delta
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

def plot_covariance_ellipse(x, y, cov, chi2=3.0, color="-r", ax=None):
    """
    This function plots an ellipse that represents a covariance matrix. The ellipse is centered at (x, y) and its shape, size and rotation are determined by the covariance matrix.

    Parameters:
    x : (float) The x-coordinate of the center of the ellipse.
    y : (float) The y-coordinate of the center of the ellipse.
    cov : (numpy.ndarray) A 2x2 covariance matrix that determines the shape, size, and rotation of the ellipse.
    chi2 : (float, optional) A scalar value that scales the ellipse size. This value is typically set based on chi-squared distribution quantiles to achieve certain confidence levels (e.g., 3.0 corresponds to ~95% confidence for a 2D Gaussian). Defaults to 3.0.
    color : (str, optional) The color and line style of the ellipse plot, following matplotlib conventions. Defaults to "-r" (a red solid line).
    ax : (matplotlib.axes.Axes, optional) The Axes object to draw the ellipse on. If None (default), a new figure and axes are created.

    Returns:
    None. This function plots the covariance ellipse on the specified axes.
    """
    eig_val, eig_vec = np.linalg.eig(cov)

    if eig_val[0] >= eig_val[1]:
        big_ind = 0
        small_ind = 1
    else:
        big_ind = 1
        small_ind = 0
    a = math.sqrt(chi2 * eig_val[big_ind])
    b = math.sqrt(chi2 * eig_val[small_ind])
    angle = math.atan2(eig_vec[1, big_ind], eig_vec[0, big_ind])
    plot_ellipse(x, y, a, b, angle, color=color, ax=ax)

def plot_ellipse(x, y, a, b, angle, color="-r", ax=None, **kwargs):
    """
    This function plots an ellipse based on the given parameters.

    Parameters
    ----------
    x : (float) The x-coordinate of the center of the ellipse.
    y : (float) The y-coordinate of the center of the ellipse.
    a : (float) The length of the semi-major axis of the ellipse.
    b : (float) The length of the semi-minor axis of the ellipse.
    angle : (float) The rotation angle of the ellipse, in radians.
    color : (str, optional) The color and line style of the ellipse plot, following matplotlib conventions. Defaults to "-r" (a red solid line).
    ax : (matplotlib.axes.Axes, optional) The Axes object to draw the ellipse on. If None (default), a new figure and axes are created.
    **kwargs: Additional keyword arguments to pass to plt.plot or ax.plot.

    Returns
    ---------
    None. This function plots the ellipse based on the specified parameters.
    """

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    px = [a * math.cos(it) for it in t]
    py = [b * math.sin(it) for it in t]
    fx = rot_mat_2d(angle) @ (np.array([px, py]))
    px = np.array(fx[0, :] + x).flatten()
    py = np.array(fx[1, :] + y).flatten()
    if ax is None:
        plt.plot(px, py, color, **kwargs)
    else:
        ax.plot(px, py, color, **kwargs)

def rot_mat_2d(angle):
    """
    Create 2D rotation matrix from an angle

    Parameters
    ----------
    angle :

    Returns
    -------
    A 2D rotation matrix

    Examples
    --------
    >>> angle_mod(-4.0)


    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]