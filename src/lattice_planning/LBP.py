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
import planner.utils as utils
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
max_acc = 10 #json_object["DWA"]["max_acc"] # [m/ss]
min_acc = -10 #json_object["DWA"]["min_acc"] # [m/ss]
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
L_d = json_object["Controller"]["L_d"]  # [m] look-ahead distance
robot_num = json_object["robot_num"]
safety_init = json_object["safety"]
width_init = json_object["width"]
height_init = json_object["height"]
min_dist = json_object["min_dist"]
N=3

show_animation = True
v_ref = 2.0 # [m/s] reference speed

with open('/home/giacomo/thesis_ws/src/lattice_planning/LBP.json', 'r') as file:
    data = json.load(file)

def lbp_control(x, goal, ob):
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

def calc_dynamic_window(x):
    """
    calculation dynamic window based on current state x
    motion model
    initial state [x(m), y(m), yaw(rad), v(m/s), delta(rad)]
    """    
    v_poss = np.arange(min_speed, max_speed+v_resolution, v_resolution)
    v_achiv = [x[3] + min_acc*dt, x[3] + max_acc*dt]

    v_search = []

    for v in v_poss:
        if v >= v_achiv[0] and v <= v_achiv[1]:
            v_search.append(v)
    
    return v_search

def calc_control_and_trajectory(x, dw, goal, ob):
    """
    calculation final input with dynamic window
    """
    x_init = x[:]
    v_search = calc_dynamic_window(x_init)
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    for v in v_search:
        dict = data[str(v)]
        for id, info in dict.items():

            # old_time = time.time()
            geom = np.zeros((len(info['x']),3))
            geom[:,0] = info['x']
            geom[:,1] = info['y']
            geom[:,2] = info['yaw']
            geom[:,0:2] = (geom[:,0:2]) @ rotateMatrix(-x[2]) + [x[0],x[1]]

            geom[:,2] = geom[:,2] + x[2] #bringing also the yaw angle in the new frame
            
            # trajectory = predict_trajectory(x_init, a, delta)
            trajectory = geom
            # calc cost

            to_goal_cost = 20 * to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            # speed_cost = speed_cost_gain * (max_speed - trajectory[-1, 3])
            ob_cost = obstacle_cost_gain * calc_obstacle_cost(trajectory, ob)
            heading_cost = heading_cost_gain * calc_to_goal_heading_cost(trajectory, goal)
            final_cost = to_goal_cost + ob_cost + heading_cost #+ speed_cost 
            
            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost

                # interpolate the control inputs
                a = (v-x[3])/dt

                best_u = [a, info['ctrl'][2]]
                best_trajectory = trajectory
    return best_u, best_trajectory #, previous_u, previous_cost

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
    if any(element < -width_init/2+WB or element > width_init/2-WB for element in x):
        return np.inf
    if any(element < -height_init/2+WB or element > height_init/2-WB for element in y):
        return np.inf

    for obstacle in ob:
        if dilated.intersects(obstacle):
            return 100000 # collision        
        elif distance(dilated, obstacle) < min_distance:
            min_distance = distance(dilated, obstacle)
            
    return 1/distance(dilated, obstacle)

def calc_to_goal_cost(trajectory, goal):
    """np.radians(90)
        calc to goal cost with angle difference
    """
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]

    cost = math.hypot(dx, dy)

    return cost

def calc_to_goal_heading_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]

    # either using the angle difference or the distance --> if we want to use the angle difference, we need to normalize the angle before taking the difference
    error_angle = normalize_angle(math.atan2(dy, dx))
    cost_angle = error_angle - normalize_angle(trajectory[-1, 2])
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


def main():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    iterations = 3000
    break_flag = False

    x = np.array([[-7, 7, 0.0], [0, 0, 7], [0, np.pi, -np.pi/2], [0, 0, 0]])
    u = np.array([[0, 0, 0], [0, 0, 0]])
    goal = np.array([[7.0, -7.0, 0], [7, 7, 0]])

    # create a trajcetory array to store the trajectory of the N robots
    trajectory = np.zeros((x.shape[0], N, 1))
    # append the firt state to the trajectory
    trajectory[:, :, 0] = x
    # trajectory = np.dstack([trajectory, x])

    # predicted_trajectory = np.zeros((N, round(predict_time/dt)+1, x.shape[0]))
    predicted_trajectory = {}

    dilated_traj = []
    for i in range(N):
        dilated_traj.append(Point(x[0, i], x[1, i]).buffer(dilation_factor, cap_style=3))

    # input [throttle, steer (delta)]
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)
    for z in range(iterations):
        for i in range(N):
            ob = []
            x1 = x[:, i]
            for idx in range(N):
                if idx == i:
                    continue
                # point = Point(x[0, idx], x[1, idx])
                # point = point.buffer(dilation_factor, cap_style=3)
                # ob.append(point)
                if utils.dist([x1[0], x1[1]], [x[0, idx], x[1, idx]]) < WB: raise Exception('Collision')
                ob.append(dilated_traj[idx])
            
            x1 = x[:, i]
            u1, predicted_trajectory1 = lbp_control(x1, goal[:,i], ob)
            line = LineString(zip(predicted_trajectory1[:, 0], predicted_trajectory1[:, 1]))
            dilated = line.buffer(dilation_factor, cap_style=3)
            dilated_traj[i] = dilated
            x1 = motion(x1, u1, dt)
            x[:, i] = x1
            u[:, i] = u1
            predicted_trajectory[i] = predicted_trajectory1
            print(f'Robot {i} v: {x1[3]}')

        trajectory = np.dstack([trajectory, x])
        
        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            for i in range(N):
                plt.plot(predicted_trajectory[i][:, 0], predicted_trajectory[i][:, 1], "-g")
                plot_polygon(dilated_traj[i], ax=ax, add_points=False, alpha=0.5)
                plt.plot(x[0,i], x[1,i], "xr")
                plt.plot(goal[0,i], goal[1,i], "xb")
                plot_robot(x[0,i], x[1,i], x[2,i])
                plot_arrow(x[0,i], x[1,i], x[2,i], length=1, width=0.5)
                plot_arrow(x[0,i], x[1,i], x[2,i]+u[1,i], length=3, width=0.5)


           
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

def main1():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    iterations = 3000
    break_flag = False

    x0, y, yaw, v, omega, model_type = utils.samplegrid(width_init, height_init, min_dist, robot_num, safety_init)
    x = np.array([x0, y, yaw, v])
    # u = np.array([[0, 0, 0], [0, 0, 0]])
    u = np.zeros((2, robot_num))

    # create a trajcetory array to store the trajectory of the robot_num robots
    trajectory = np.zeros((x.shape[0], robot_num, 1))
    # append the firt state to the trajectory
    trajectory[:, :, 0] = x
    # trajectory = np.dstack([trajectory, x])

    # predicted_trajectory = np.zeros((N, round(predict_time/dt)+1, x.shape[0]))
    predicted_trajectory = {}

    paths = []
    targets = []
    dilated_traj = []
    for i in range(robot_num):
        dilated_traj.append(Point(x[0, i], x[1, i]).buffer(dilation_factor, cap_style=3))
        paths.append(utils.create_path())
        targets.append([paths[i][0].x, paths[i][0].y])

    # input [throttle, steer (delta)]
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)

    for z in range(iterations):
        for i in range(robot_num):
            # Updating the paths of the robots
            if utils.dist(point1=(x[0,i], x[1,i]), point2=targets[i]) < 5:
                # get_logger().info("Updating the path for robot " + str(i))
                paths[i] = utils.update_path(paths[i])
                targets[i] = (paths[i][0].x, paths[i][0].y)
            
            x1 = x[:, i]

            ob = []
            for idx in range(robot_num):
                if idx == i:
                    continue
                if utils.dist([x1[0], x1[1]], [x[0, idx], x[1, idx]]) < WB: raise Exception('Collision')
    
                ob.append(dilated_traj[idx])

            u1, predicted_trajectory1 = lbp_control(x1, targets[i], ob)
            line = LineString(zip(predicted_trajectory1[:, 0], predicted_trajectory1[:, 1]))
            dilated = line.buffer(dilation_factor, cap_style=3)
            dilated_traj[i] = dilated
            x1 = motion(x1, u1, dt)
            x[:, i] = x1
            u[:, i] = u1
            predicted_trajectory[i] = predicted_trajectory1
            print(f'Robot {i} v: {x1[3]}')

        trajectory = np.dstack([trajectory, x])
        
        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            for i in range(robot_num):
                plt.plot(predicted_trajectory[i][:, 0], predicted_trajectory[i][:, 1], "-g")
                plot_polygon(dilated_traj[i], ax=ax, add_points=False, alpha=0.5)
                plt.plot(x[0,i], x[1,i], "xr")
                plt.plot(targets[i][0], targets[i][1], "xg")
                plot_robot(x[0,i], x[1,i], x[2,i])
                plot_arrow(x[0,i], x[1,i], x[2,i], length=1, width=0.5)
                plot_arrow(x[0,i], x[1,i], x[2,i]+u[1,i], length=3, width=0.5)

            utils.plot_map(width=width_init, height=height_init)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
        
        for i in range(robot_num):
            dist_to_goal = math.hypot(x[0,i] - targets[i][0], x[1,i] - targets[i][1])
            if dist_to_goal <= 0.5:
                print("Goal!!")
                break_flag = True
        
        if break_flag:
            break
        
    print("Done")
    if show_animation:
        for i in range(robot_num):
            plt.plot(trajectory[0,i,:], trajectory[1,i,:], "-r")
        plt.pause(0.0001)
        plt.show()
       
if __name__ == '__main__':
    main1()
    # main()

    