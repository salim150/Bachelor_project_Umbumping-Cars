import numpy as np
import cubic_spline_planner
import math
import random
import sys
sys.path.append("..")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from scipy.optimize import minimize, NonlinearConstraint
import time
import planner.utils as utils

import pathlib
import json

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
L_d = json_object["Controller"]["L_d"]  # [m] look-ahead distance
robot_num = json_object["robot_num"]
safety_init = json_object["safety"]
width_init = json_object["width"]
height_init = json_object["height"]
min_dist = json_object["min_dist"]
# N=3

timer_freq = json_object["timer_freq"]

show_animation = True
debug = False

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 3.0 # maximum speed [m/s]
MIN_SPEED = -3.0  # minimum speed [m/s]
MAX_ACCEL = 5.0  # maximum accel [m/ss]

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

class ModelPredictiveControl:
    def __init__(self, obs_x, obs_y):
        self.horizon = 8
        self.dt = 0.2

        # self.L = 2.5 # Car base [m]

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = obs_x
        self.y_obs = obs_y

        self.initial_state = None
        self.safety_radius = 2.5

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t += np.cos(psi_t) * v_t * dt
        y_t += np.sin(psi_t) * v_t * dt

        a_t = pedal
        v_t += a_t * dt #- v_t/25
        v_t = np.clip(v_t, min_speed, max_speed)

        psi_t += v_t * dt * np.tan(steering)/L
        psi_t = normalize_angle(psi_t)

        return [x_t, y_t, psi_t, v_t]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(self.horizon):
            speed = state[3]
            heading = state[2]

            state = self.plant_model(state, self.dt, u[i*2], u[i*2 + 1])

            distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)

            # Position cost
            # cost +=  distance_to_goal

            # Obstacle cost
            for z in range(len(self.x_obs)-1):
                distance_to_obstacle = np.sqrt((self.x_obs[z] - state[0])**2 + (self.y_obs[z] - state[1])**2)
                # if any(distance_to_obstacle < 3):
                if distance_to_obstacle < 5:
                    cost += 100 #np.inf/distance_to_obstacle

            # Heading cost
            cost += 10 * (heading - state[2])**2
            # cost += 10 * state[2]

            # negative speed cost
            # cost += -5 * np.sign(speed) * 3 * speed

            # Acceleration cost
            # if abs(u[2*i]) > 0.2:
            #     cost += (speed - state[3])**2

        # Final heading and position cost        
        # cost +=  4 * (normalize_angle(ref[2]) - normalize_angle(state[2]))**2
        distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)
        cost += 5*distance_to_goal
        print(f'cost: {cost}, distance to goal: {distance_to_goal}')
        return cost
    
    def cost_function2(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(self.horizon):
            speed = state[3]
            heading = state[2]

            state = self.plant_model(state, self.dt, u[i*2], u[i*2 + 1])

            distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)

            # Position cost
            cost +=  distance_to_goal

            # Obstacle cost
            for z in range(len(self.x_obs)-1):
                distance_to_obstacle = np.sqrt((self.x_obs[z] - state[0])**2 + (self.y_obs[z] - state[1])**2)
                if distance_to_obstacle < 3:
                    cost += 40/distance_to_obstacle

            # Heading cost
            cost += 10 * (heading - state[2])**2

            # cost +=  2 * (ref[2] - state[2])**2

            # negative speed cost
            cost += -10 * np.sign(speed) * 3 * speed

            # Acceleration cost
            if abs(u[2*i]) > 0.2:
                cost += (speed - state[3])**2

        cost += (state[3])**2
        distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)
        cost += 100*distance_to_goal
        return cost

    def cost_function3(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(self.horizon):
            speed = state[3]
            heading = state[2]

            state = self.plant_model(state, self.dt, u[i*2], u[i*2 + 1])

            distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)

            # Position cost
            cost +=  distance_to_goal

            # Obstacle cost
            # for z in range(len(self.x_obs)-1):
            #     distance_to_obstacle = np.sqrt((self.x_obs[z] - state[0])**2 + (self.y_obs[z] - state[1])**2)
            #     if distance_to_obstacle < 3:
            #         cost += 40/distance_to_obstacle

            # Heading cost
            cost += 10 * (heading - state[2])**2

            cost +=  2 * (ref[2] - state[2])**2

            # negative speed cost
            cost += -10 * np.sign(speed) * 3 * speed

            # Acceleration cost
            if abs(u[2*i]) > 0.2:
                cost += (speed - state[3])**2

        cost += (state[3])**2
        distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)
        cost += 100*distance_to_goal
        return cost

    def propagation1(self, u):
        state = [self.initial_state]

        for i in range(self.horizon):
            state.append(self.plant_model(state[-1], self.dt, u[2*i], u[2*i + 1]))
        return np.array(state)[:,0]
    
    def propagation2(self, u):
        state = [self.initial_state]

        for i in range(self.horizon):
            state.append(self.plant_model(state[-1], self.dt, u[2*i], u[2*i + 1]))
        return np.array(state)[:,1]
    
    def propagation3(self, u):
        state = self.initial_state
        distance = []

        for t in range(self.horizon):
            state = self.plant_model(state, self.dt, u[2*t], u[2*t + 1])
            for i in range(len(self.x_obs)):
                distance.append((state[0] - self.x_obs[i])**2 + (state[1] - self.y_obs[i])**2-self.safety_radius**2)

        return np.array(distance)

def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck

def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def get_straight_course(start, goal, dl):
    ax = [start[0], goal[0]]
    ay = [start[1], goal[1]]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    return cx, cy, cyaw, ck
    
def update_paths(i, x, cx, cy, cyaw, target_ind, ref, dl):
    x1 = x[:, i]
    # Updating the paths of the robots
    if (target_ind[i] < len(cx[i])-1):
        if dist([x1[0], x1[1]], [cx[i][target_ind[i]], cy[i][target_ind[i]]]) < 4:
            target_ind[i]+=1
            ref[i][0] = cx[i][target_ind[i]]
            ref[i][1] = cy[i][target_ind[i]]
            ref[i][2] = cyaw[i][target_ind[i]]
    elif (target_ind[i] == len(cx[i])-1):
        target_ind[i] = 0
        cx.pop(i)
        cy.pop(i)
        cyaw.pop(i)
        sample_point = (float(random.randint(-width_init/2, width_init/2)), float(random.randint(-height_init/2, height_init/2)))
        cx.insert(i, get_straight_course(start=(x[0,i], x[1,i]), goal=(sample_point[0], sample_point[1]), dl=dl)[0])
        cy.insert(i, get_straight_course(start=(x[0,i], x[1,i]), goal=(sample_point[0], sample_point[1]), dl=dl)[1])
        cyaw.insert(i, get_straight_course(start=(x[0,i], x[1,i]), goal=(sample_point[0], sample_point[1]), dl=dl)[2])
        
        ref[i] = [cx[i][target_ind[i]], cy[i][target_ind[i]], cyaw[i][target_ind[i]]]
    
    return cx, cy, ref

def update_obstacles(mpc, i, x1, x, predicted_trajectory):
        mpc.x_obs = []
        mpc.y_obs = []
        for idx in range(robot_num):
            if idx == i:
                continue
            if dist([x1[0], x1[1]], [x[0, idx], x[1, idx]]) < 1:
                raise Exception('Collision')
            
            next_robot_state = predicted_trajectory[idx]
            mpc.x_obs.append(next_robot_state[0:-1:5, 0])
            mpc.y_obs.append(next_robot_state[0:-1:5, 1])
        mpc.x_obs = [item for sublist in mpc.x_obs for item in sublist]
        mpc.y_obs = [item for sublist in mpc.y_obs for item in sublist]

def generate_reference_trajectory(x, dl):
    cx = []
    cy = []
    cyaw = []
    ref = []
    target_ind = [1] * robot_num
    for i in range(robot_num):
        sample_point = (float(random.randint(-width_init/2, width_init/2)), float(random.randint(-height_init/2, height_init/2)))

        cx.append(get_straight_course(start=(x[0, i], x[1, i]), goal=(sample_point[0], sample_point[1]), dl=dl)[0])
        cy.append(get_straight_course(start=(x[0, i], x[1, i]), goal=(sample_point[0], sample_point[1]), dl=dl)[1])
        cyaw.append(get_straight_course(start=(x[0, i], x[1, i]), goal=(sample_point[0], sample_point[1]), dl=dl)[2])

        ref.append([cx[i][target_ind[i]], cy[i][target_ind[i]], cyaw[i][target_ind[i]]])

        if debug:
            plt.plot(x[0, i], x[1, i], "xr")
            plt.plot(cx[i], cy[i], "-r", label="course")

    if debug:
        plt.show()
    return cx, cy, cyaw, ref, target_ind

def set_bounds_and_constraints(mpc):
        bounds = []
        # Set bounds for inputs bounded optimization.
        for i in range(mpc.horizon):
            bounds += [[min_acc, max_acc]]
            bounds += [[-max_steer, max_steer]]

        constraint1 = NonlinearConstraint(fun=mpc.propagation1, lb=-width_init/2 + mpc.safety_radius, ub=width_init/2 - mpc.safety_radius)
        constraint2 = NonlinearConstraint(fun=mpc.propagation2, lb=-height_init/2 + mpc.safety_radius, ub=height_init/2 - mpc.safety_radius)
        constraint3 = NonlinearConstraint(fun=mpc.propagation3, lb=0, ub=np.inf)
        constraints = [constraint1, constraint2, constraint3]

        return bounds, constraints

def mpc_control(i, x, u, mpc, bounds, constraints, ref, predicted_trajectory):
    x1 = x[:, i]
    u1 = u[:,i]
    u1 = np.delete(u1,0)
    u1 = np.delete(u1,0)
    u1 = np.append(u1, u1[-2])
    u1 = np.append(u1, u1[-2])

    update_obstacles(mpc, i, x1, x, predicted_trajectory)        

    mpc.initial_state = x1
    # MPC control
    u_solution = minimize(mpc.cost_function3, u1, (x1, ref[i]),
                        method='SLSQP',
                        bounds=bounds,
                        constraints=constraints,
                        tol = 1e-3)
    
    u1 = u_solution.x
    x1 = mpc.plant_model(x1, dt, u1[0], u1[1])
    x[:, i] = x1
    u[:, i] = u1
    predicted_state = np.array([x1])

    for j in range(1, mpc.horizon):
        predicted = mpc.plant_model(predicted_state[-1], mpc.dt, u1[2*j], u1[2*j+1])
        predicted_state = np.append(predicted_state, np.array([predicted]), axis=0)
    predicted_trajectory[i] = predicted_state

    return x, u, predicted_trajectory

def main():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    iterations = 3000
    break_flag = False
    dl = 3
    
    # MPC initialization
    mpc = ModelPredictiveControl(obs_x=[], obs_y=[])
    
    x0, y, yaw, v, omega, model_type = utils.samplegrid(width_init, height_init, min_dist, robot_num, safety_init)
    x = np.array([x0, y, yaw, v])
    num_inputs = 2
    u = np.zeros([mpc.horizon*num_inputs, robot_num])

    # Generate reference trajectory
    cx, cy, cyaw, ref, target_ind = generate_reference_trajectory(x, dl)

    # Usage:
    bounds, constraints = set_bounds_and_constraints(mpc)

    # predicted_trajectory = np.zeros((robot_num, mpc.horizon, x.shape[0]))
    # for i in range(robot_num):
    #     predicted_trajectory[i, :, :] = x[:, i] 
    
    predicted_trajectory = dict.fromkeys(range(robot_num),np.zeros([mpc.horizon, x.shape[0]]))
    for i in range(robot_num):
        predicted_trajectory[i] = np.full((mpc.horizon, 4), x[:,i])
    
    # input [throttle, steer (delta)]
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)

    for z in range(iterations):
        plt.cla()
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        for i in range(robot_num):
            start_time = time.time()
            cx, cy, ref = update_paths(i, x, cx, cy, cyaw, target_ind, ref, dl)
            x, u, predicted_trajectory = mpc_control(i, x, u, mpc, bounds, constraints, ref, predicted_trajectory)
            
            if debug:
                print('Robot ' + str(i+1) + ' of ' + str(robot_num) + '   Time ' + str(round(time.time() - start_time,5)))

            utils.plot_robot(x[0, i], x[1, i], x[2,i])
            plt.plot(ref[i][0],ref[i][1], "xg")
            plt.plot(cx[i], cy[i], "-r", label="course")
            plt.plot(predicted_trajectory[i][:,0], predicted_trajectory[i][:, 1], "-g")
            
        plt.title('MPC 2D')
        utils.plot_map(width=width_init, height=height_init)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

if __name__ == '__main__':
    # main()
    # main2()
    main()