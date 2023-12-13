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
# for debugging
import time

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
heading_cost_gain = json_object["DWA"]["heading_cost_gain"]
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


with open('/home/giacomo/thesis_ws/src/trajectories.json', 'r') as file:
    data = json.load(file)

def dwa_control(x, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x)
    u, trajectory = calc_control_and_trajectory(x, dw, goal, ob)
    return u, trajectory

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

def rotateMatrix(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

def calc_dynamic_window(x):
    """
    calculation dynamic window based on current state x
    motion model
    initial state [x(m), y(m), yaw(rad), v(m/s), delta(rad)]
    """
    # Dynamic window from robot specification
    # Vs = [min_speed, max_speed,
    #       -max_steer, max_steer]
    
    
    # # Dynamic window from motion model
    # Vd = [x[3] - config.max_acc*0.1,
    #       x[3] + config.max_acc*0.1,
    #       -max_steer,
    #       max_steer]
    
    # #  [min_throttle, max_throttle, min_steer, max_steer]
    # dw = [min(Vs[0], Vd[0]), min(Vs[1], Vd[1]), min(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    
    Vs = [min_acc, max_acc,
          -max_steer, max_steer]
    
    # Vd = [(min_speed - x[3])/0.1 , (max_speed - x[3])/0.1,
    #       -max_steer, max_steer]
    
    dw = [Vs[0], Vs[1], Vs[2], Vs[3]]
    # dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw

def predict_trajectory(x_init, a, delta):
    """
    predict trajectory with an input
    motion model
    initial state [x(m), y(m), yaw(rad), v(m/s), delta(rad)]
    """
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time < predict_time:
        x = motion(x, [a, delta], dt)
        trajectory = np.vstack((trajectory, x))
        time += dt
    return trajectory

def calc_control_and_trajectory(x, dw, goal, ob):
    """
    calculation final input with dynamic window
    """
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    # old_time = time.time()
    for a in np.arange(dw[0], dw[1]+v_resolution, v_resolution):
        for delta in np.arange(dw[2], dw[3]+delta_resolution, delta_resolution):

            # old_time = time.time()
            nearest = find_nearest(np.arange(min_speed, max_speed, v_resolution), x[3])
            geom = data[str(nearest)][str(a)][str(delta)]
            geom = np.array(geom)
            geom[:,0:2] = (geom[:,0:2]) @ rotateMatrix(np.radians(90)-x[2]) + [x[0],x[1]]
            # print(time.time()-old_time)

            # trajectory = predict_trajectory(x_init, a, delta)
            trajectory = geom
            # calc cost

            to_goal_cost = to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = speed_cost_gain * (max_speed - trajectory[-1, 3])
            ob_cost = obstacle_cost_gain * calc_obstacle_cost(trajectory, ob)
            heading_cost = heading_cost_gain * calc_to_goal_heading_cost(trajectory, goal)
            final_cost = to_goal_cost + ob_cost# + speed_cost + heading_cost 
            
            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [a, delta]
                best_trajectory = trajectory
                if abs(best_u[0]) < robot_stuck_flag_cons \
                        and abs(x[2]) < robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -max_steer
    # print(time.time()-old_time)
    return best_u, best_trajectory
def calc_obstacle_cost(trajectory, ob):
    """
    calc obstacle cost inf: collision
    """
    line = LineString(zip(trajectory[:, 0], trajectory[:, 1]))
    dilated = line.buffer(dilation_factor, cap_style=3)

    min_distance = np.inf

    x = trajectory[:, 0]
    y = trajectory[:, 1]

    # check if the trajectory is out of bounds
    if any(element < -width_init/2 or element > width_init/2 for element in x):
        return np.inf
    if any(element < -height_init/2 or element > height_init/2 for element in y):
        return np.inf

    for obstacle in ob:
        if dilated.intersects(obstacle):
            return 100000 # collision        
        elif distance(dilated, obstacle) < min_distance:
            min_distance = distance(dilated, obstacle)
            
    return 1/distance(dilated, obstacle)

def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]

    # either using the angle difference or the distance --> if we want to use the angle difference, we need to normalize the angle before taking the difference
    # error_angle = math.atan2(dy, dx)
    # cost_angle = error_angle - trajectory[-1, 2]
    # cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    cost = math.hypot(dx, dy)
    return cost

def calc_to_goal_heading_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]

    # either using the angle difference or the distance --> if we want to use the angle difference, we need to normalize the angle before taking the difference
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost

def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_robot(x, y, yaw):  # pragma: no cover
    if robot_type == RobotType.rectangle:
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
    elif robot_type == RobotType.circle:
        circle = plt.Circle((x, y), robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")


def main(gx=10.0, gy=30.0, robot_type=RobotType.rectangle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    iterations = 3000
    break_flag = False
    x = np.array([[0, 10, 10], [0, 0, 10], [0, np.pi, -np.pi/2], [0, 0, 0]])
    goal = np.array([[30, 0, 15], [10, 10, 0]])
    # x = np.array([[0, 20], [0, 0], [0, np.pi], [0, 0]])
    # goal = np.array([[30, 0], [10, 10]])

    # goal2 = np.array([0, 10])
    cmd1 = ControlInputs()
    cmd2 = ControlInputs()

    # create a trajcetory array to store the trajectory of the N robots
    trajectory = np.zeros((x.shape[0], N, 1))
    # append the firt state to the trajectory
    trajectory[:, :, 0] = x
    # trajectory = np.dstack([trajectory, x])

    # predicted_trajectory = np.zeros((N, round(predict_time/dt)+1, x.shape[0]))
    predicted_trajectory = np.zeros((N, 12, x.shape[0]))

    dilated_traj = []
    for i in range(N):
        dilated_traj.append(Point(x[0, i], x[1, i]).buffer(dilation_factor, cap_style=3))

    # input [throttle, steer (delta)]
    robot_type = robot_type
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)
    for z in range(iterations):
        for i in range(N):
            ob = []
            for idx in range(N):
                if idx == i:
                    continue
                # point = Point(x[0, idx], x[1, idx])
                # point = point.buffer(dilation_factor, cap_style=3)
                # ob.append(point)
                ob.append(dilated_traj[idx])
            
            x1 = x[:, i]
            u1, predicted_trajectory1 = dwa_control(x1, goal[:,i], ob)
            line = LineString(zip(predicted_trajectory1[:, 0], predicted_trajectory1[:, 1]))
            dilated = line.buffer(dilation_factor, cap_style=3)
            dilated_traj[i] = dilated
            x1 = motion(x1, u1, dt)
            x[:, i] = x1
            predicted_trajectory[i, :, :] = predicted_trajectory1

        trajectory = np.dstack([trajectory, x])
        
        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            for i in range(N):
                plt.plot(predicted_trajectory[i, :, 0], predicted_trajectory[i, :, 1], "-g")
                plot_polygon(dilated_traj[i], ax=ax, add_points=False, alpha=0.5)
                plt.plot(x[0,i], x[1,i], "xr")
                plt.plot(goal[0,i], goal[1,i], "xb")
                plot_robot(x[0,i], x[1,i], x[2,i])
                plot_arrow(x[0,i], x[1,i], x[2,i], length=2, width=1)

           
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
       
        for i in range(N):
            dist_to_goal = math.hypot(x[0,i] - goal[0,i], x[1,i] - goal[1,i])
            if dist_to_goal <= 1:
                print("Goal!!")
                break_flag = True
        
        if break_flag:
            break
        
    print("Done")
    if show_animation:
        for i in range(N):
            plt.plot(trajectory[0,i,:], trajectory[1,i,:], "-r")
        plt.pause(0.0001)
        plt.show()
       
if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)