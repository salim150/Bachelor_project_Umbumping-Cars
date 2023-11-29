import matplotlib.pyplot as plt
import numpy as np
import math
from enum import Enum
# For the parameter file
import pathlib
import json
from custom_message.msg import ControlInputs
from shapely.geometry import Point, Polygon, LineString
from shapely import intersection, distance
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

def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)
    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)
    return u, trajectory

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
        self.max_speed = 2 # [m/s]
        self.min_speed = -2# [m/s]
        self.max_delta = np.radians(45)  # [rad]
        self.max_accel = 0.2  # [m/ss]
        # self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.5# [m/s]
        self.delta_resolution = math.radians(5)  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 0.7  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 70.0
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

def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    motion model
    initial state [x(m), y(m), yaw(rad), v(m/s), delta(rad)]
    """
    # Dynamic window from robot specification
    Vs = [min_speed, max_speed,
          -max_steer, max_steer]
    
    
    # Dynamic window from motion model
    Vd = [x[3] - config.max_acc*0.1,
          x[3] + config.max_acc*0.1,
          -max_steer,
          max_steer]
    
    #  [min_throttle, max_throttle, min_steer, max_steer]
    dw = [min(Vs[0], Vd[0]), min(Vs[1], Vd[1]), min(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    
    return dw

def predict_trajectory(x_init, a, delta, config):
    """
    predict trajectory with an input
    motion model
    initial state [x(m), y(m), yaw(rad), v(m/s), delta(rad)]
    """
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [a, delta], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt
    return trajectory

def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])
    # evaluate all trajectory with sampled input in dynamic window
    for a in np.arange(dw[0], dw[1], config.v_resolution):
        for delta in np.arange(dw[2], dw[3], config.delta_resolution):
            trajectory = predict_trajectory(x_init, a, delta, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            final_cost = to_goal_cost + speed_cost + ob_cost
            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [a, delta]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[2]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -max_steer
    return best_u, best_trajectory
def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    line = LineString(zip(trajectory[:, 0], trajectory[:, 1]))
    dilated = line.buffer(0.5, cap_style=3)
    
    if intersection(dilated, ob):
        return float("Inf")
    else:
        return 1/distance(dilated, ob)
    
    # ob = ob.reshape(ob.shape[0], N-1)
    # ox = ob[0, :]
    # oy = ob[1, :]
    # dx = trajectory[:, 0] - ox[:, None]
    # dy = trajectory[:, 1] - oy[:, None]
    # r = np.hypot(dx, dy)

    # if config.robot_type == RobotType.rectangle:
    #     yaw = trajectory[:, 2]
    #     rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    #     rot = np.transpose(rot, [2, 0, 1])
    #     local_ob = ob[:, None] - trajectory[:, 0:2]
    #     local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    #     local_ob = np.array([local_ob @ x for x in rot])
    #     local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    #     upper_check = local_ob[:, 0] <= L / 2
    #     right_check = local_ob[:, 1] <= L / 2
    #     bottom_check = local_ob[:, 0] >= - L / 2
    #     left_check = local_ob[:, 1] >= - L / 2
    #     if (np.logical_and(np.logical_and(upper_check, right_check),
    #                        np.logical_and(bottom_check, left_check))).any():
    #         return float("Inf")
    # elif config.robot_type == RobotType.circle:
    #     if np.array(r <= config.robot_radius).any():
    #         return float("Inf")

    # min_r = np.min(r)
    # return 1.0 / min_r  # OK

def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
    return cost

def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-L / 2, L / 2,
                             (L / 2), -L / 2,
                             -L / 2],
                            [WB / 2, WB / 2,
                             - WB / 2, -WB / 2,
                             WB / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")


def main(gx=10.0, gy=30.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    # x = np.array([0.0, 0.0, math.pi / 8.0, 1.0, 0.0])
    iterations = 3000

    x = np.array([[0, 20], [0, 0], [0, np.pi], [0, 0]])
    goal1 = np.array([20, 10])
    goal2 = np.array([0, 10])
    cmd1 = ControlInputs()
    cmd2 = ControlInputs()

    trajectory1 = np.array(x[:,0])
    trajectory2 = np.array(x[:,1])
    # input [throttle, steer (delta)]
    config.robot_type = robot_type
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)
    for i in range(iterations):
        ob = []
        for i in range(N):
            line = Point(x[0, i], x[1, i])
            line = line.buffer(0.5, cap_style=3)
            ob.append(line)
        
        # ob = x[:2,:]
        x1 = x[:, 0]
        x2 = x[:, 1]
        u1, predicted_trajectory1 = dwa_control(x1, config, goal1, ob[1])
        line = LineString(zip(predicted_trajectory1[:, 0], predicted_trajectory1[:, 1]))
        dilated = line.buffer(1, cap_style=3)
        ob.append(dilated)
        u2, predicted_trajectory2 = dwa_control(x2, config, goal2, ob[2])
        x1 = motion(x1, u1, config.dt)  # simulate robot
        x2 = motion(x2, u2, config.dt)  # simulate robot
        x = np.concatenate((x1[:, None], x2[:, None]), axis=1)

        trajectory1 = np.vstack((trajectory1, x1))
        trajectory2 = np.vstack((trajectory2, x2))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory1[:, 0], predicted_trajectory1[:, 1], "-g")
            plt.plot(predicted_trajectory2[:, 0], predicted_trajectory2[:, 1], "-g")
            plot_polygon(dilated, ax=ax, add_points=False, alpha=0.5)
            plt.plot(x1[0], x1[1], "xr")
            plt.plot(x2[0], x2[1], "xg")
            plt.plot(goal1[0], goal1[1], "xb")
            plt.plot(goal2[0], goal2[1], "xk")

            plot_robot(x1[0], x1[1], x1[2], config)
            plot_arrow(x1[0], x1[1], x1[2])
            plot_robot(x2[0], x2[1], x2[2], config)
            plot_arrow(x2[0], x2[1], x2[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
        # check reaching goal
        dist_to_goal1 = math.hypot(x1[0] - goal1[0], x1[1] - goal1[1])
        dist_to_goal2 = math.hypot(x2[0] - goal2[0], x2[1] - goal2[1])
        if dist_to_goal1 <= 5 or dist_to_goal2 <= 5:
            print("Goal!!")
            break
    print("Done")
    if show_animation:
        plt.plot(trajectory1[:, 0], trajectory1[:, 1], "-r")
        plt.plot(trajectory2[:, 0], trajectory2[:, 1], "-r")
        plt.pause(0.0001)
        plt.show()
if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    # main(robot_type=RobotType.circle)