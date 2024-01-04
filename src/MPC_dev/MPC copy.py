import numpy as np
from sim2d import sim_run
import cubic_spline_planner
import math
import random
import sys
sys.path.append('/home/giacomo/thesis_ws/src/bumper_cars/src')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from scipy.optimize import minimize, NonlinearConstraint
import time
# from planner.utils import *
# from planner import *
import planner.utils as utils
# import lanner.utils as utils

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
robot_num = 2 #json_object["robot_num"]
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
        self.horizon = 10
        self.dt = 0.2

        # self.L = 2.5 # Car base [m]

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = obs_x
        self.y_obs = obs_y

        self.initial_state = None
        self.safety_radius = 3.0

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

    def cost_function(self,u, *args):
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
                if distance_to_obstacle < 1:
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

            state = self.plant_model(state, self.dt, u[i*3], u[i*3 + 1])

            distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)

            # Position cost
            cost +=  distance_to_goal

            # Obstacle cost
            for z in range(len(self.x_obs)-1):
                distance_to_obstacle = np.sqrt((self.x_obs[z] - state[0])**2 + (self.y_obs[z] - state[1])**2)
                # if any(distance_to_obstacle < 3):
                if distance_to_obstacle < 4:
                    cost += 1000 #np.inf/distance_to_obstacle

            # Heading cost
            cost += 1 * (heading - state[2])**2

            # negative speed cost
            cost += -10 * np.sign(speed) * 3 * speed

            # Acceleration cost
            if abs(u[2*i]) > 0.2:
                cost += (speed - state[3])**2

        # Final heading and position cost        
        cost +=  10 * (normalize_angle(ref[2]) - normalize_angle(state[2]))**2
        distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)
        cost += 10*distance_to_goal
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
            #     if distance_to_obstacle < 1:
            #         cost += 40/distance_to_obstacle

            # Heading cost
            cost += 10 * (heading - state[2])**2

            cost +=  2 * (ref[2] - state[2])**2

            # Acceleration cost
            if abs(u[2*i]) > 0.2:
                cost += (speed - state[3])**2

        cost += (state[3])**2
        distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)
        cost += 50*distance_to_goal
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
    
def main():
    print(__file__ + " start!!")

    dl = 1.0  
    cx, cy, cyaw, ck = get_switch_back_course(dl)

    for i in range(len(cyaw)):
        cyaw[i] = normalize_angle(cyaw[i])

    initial_state = np.array([cx[0], cy[0], cyaw[0], 0.0])
    
    # sim_run(options, ModelPredictiveControl, initial_state, cx, cy, cyaw, ck)

    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    OBSTACLES = options['OBSTACLES']

    mpc = ModelPredictiveControl(obs_x=cx[5:-1:12], obs_y=cy[5:-1:12])

    num_inputs = 2
    u = np.zeros(mpc.horizon*num_inputs)
    bounds = []

    # Set bounds for inputs bounded optimization.
    for i in range(mpc.horizon):
        bounds += [[min_acc, max_acc]]
        bounds += [[-max_steer, max_steer]]
    
    target_ind = 1
    ref = [cx[target_ind], cy[target_ind], cyaw[target_ind]]

    state_i = np.array([initial_state])
    u_i = np.array([[0,0]])
    sim_total = 1000
    predict_info = [state_i]

    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(8,8)

    # Elevator plot settings.
    ax = fig.add_subplot(gs[:8, :8])

    plt.xlim(-3, 17)
    ax.set_ylim([-3, 17])
    plt.xticks(np.arange(0,11, step=2))
    plt.yticks(np.arange(0,11, step=2))
    plt.title('MPC 2D')

    for i in range(1,sim_total+1):
        u = np.delete(u,0)
        u = np.delete(u,0)
        u = np.append(u, u[-2])
        u = np.append(u, u[-2])
        start_time = time.time()

        # explore possibility of iterative MPC: for z in range(3):
        # Non-linear optimization.
        u_solution = minimize(mpc.cost_function3, u, (state_i[-1], ref),
                                method='SLSQP',
                                bounds=bounds,
                                tol = 1e-2)
        print('Step ' + str(i) + ' of ' + str(sim_total) + '   Time ' + str(round(time.time() - start_time,5)))
        u = u_solution.x
        y = mpc.plant_model(state_i[-1], mpc.dt, u[0], u[1])
        if (target_ind < len(cx)-1):
            if dist([y[0], y[1]], [cx[target_ind], cy[target_ind]]) < 4:
                target_ind+=1
                ref[0] = cx[target_ind]
                ref[1] = cy[target_ind]
                ref[2] = cyaw[target_ind]

        predicted_state = np.array([y])
        for j in range(1, mpc.horizon):
            if u[2*j]>max_acc or u[2*j]<min_acc:
                print('Acceleration out of bounds')
                break
            elif u[2*j+1]>max_steer or u[2*j+1]<-max_steer:
                print('Steering out of bounds')
                break
            predicted = mpc.plant_model(predicted_state[-1], mpc.dt, u[2*j], u[2*j+1])
            predicted_state = np.append(predicted_state, np.array([predicted]), axis=0)
        predict_info += [predicted_state]
        state_i = np.append(state_i, np.array([y]), axis=0)
        # print(f'yaw angle: {y[2]}')
        # print(f'speed: {y[3]}')
        # print(f'ref yaw angle: {ref[2]}')
        u_i = np.append(u_i, np.array([(u[0], u[1])]), axis=0)

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if OBSTACLES:
            for zz in range(len(mpc.x_obs)):
                patch_obs = mpatches.Circle((mpc.x_obs[zz], mpc.y_obs[zz]),0.5)
                ax.add_patch(patch_obs)
        utils.plot_robot(state_i[i,0], state_i[i,1], state_i[i,2])
        utils.plot_robot(ref[0],ref[1],ref[2])
        plt.plot(cx, cy, "-r", label="course")
        plt.plot(predicted_state[:,0], predicted_state[:,1])
        plt.xlim(-10, 40)
        plt.ylim(-10, 40)
        plt.title('MPC 2D')
        plt.grid(True)
        plt.pause(0.0001)

def main4():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    iterations = 3000
    break_flag = False
    dl = 2
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    OBSTACLES = options['OBSTACLES']
    
    x0, y, yaw, v, omega, model_type = utils.samplegrid(width_init, height_init, min_dist, robot_num, safety_init)
    x = np.array([x0, y, yaw, v])

    cx = []
    cy = []
    cyaw = []
    ref = []
    target_ind = [1]*robot_num
    for i in range(robot_num):
        sample_point = (float(random.randint(-width_init/2, width_init/2)), float(random.randint(-height_init/2, height_init/2)))
    
        cx.append(get_straight_course(start=(x[0,i], x[1,i]), goal=(sample_point[0], sample_point[1]), dl=dl)[0])
        cy.append(get_straight_course(start=(x[0,i], x[1,i]), goal=(sample_point[0], sample_point[1]), dl=dl)[1])
        cyaw.append(get_straight_course(start=(x[0,i], x[1,i]), goal=(sample_point[0], sample_point[1]), dl=dl)[2])
   
        ref.append([cx[i][target_ind[i]], cy[i][target_ind[i]], cyaw[i][target_ind[i]]])
    
        plt.plot(x[0,i], x[1,i], "xr")
        # plt.plot(targets[i][0], targets[i][1], "xg")
        plt.plot(cx[i], cy[i], "-r", label="course")
    
    plt.show()

    # MPC initialization
    mpc = ModelPredictiveControl(obs_x=[0,7,5,-2,-4,8,9], obs_y=[0,-6,3,7,-3,5,9])
    # mpc = ModelPredictiveControl(obs_x=cx[0][5:-1:12], obs_y=cy[0][5:-1:12])

    num_inputs = 2
    u = np.zeros([mpc.horizon*num_inputs, robot_num])
    bounds = []

    # Set bounds for inputs bounded optimization.
    for i in range(mpc.horizon):
        bounds += [[min_acc, max_acc]]
        bounds += [[-max_steer, max_steer]]

    predicted_trajectory = np.zeros((robot_num, mpc.horizon, x.shape[0]))
    
    # input [throttle, steer (delta)]
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)

    for z in range(iterations):
        # old_time = time.time()
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        for i in range(robot_num):
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
            
            u1 = u[:,i]
            u1 = np.delete(u1,0)
            u1 = np.delete(u1,0)
            u1 = np.append(u1, u1[-2])
            u1 = np.append(u1, u1[-2])
            start_time = time.time()


            mpc.x_obs = []
            mpc.y_obs = []
            for idx in range(robot_num):
                if idx == i:
                    continue
                # ob.append([x[0, idx], x[1, idx]])
                mpc.x_obs.append(x[0, idx])
                mpc.y_obs.append(x[1, idx])
                # ob.append([predicted_trajectory[i, :, 0].tolist(), predicted_trajectory[i, :, 1].tolist()])
                # add only if the robots are close enough

                # if dist([x1[0], x1[1]], [x[0, idx], x[1, idx]]) < 10:
                # if dist([x1[0], x1[1]], [x[0, idx], x[1, idx]]) < 1: raise Exception('Collision')
                # mpc.x_obs.append(predicted_trajectory[i, 0:-1:5, 0])
                # mpc.y_obs.append(predicted_trajectory[i, 0:-1:5, 1])
            # mpc.x_obs = [item for sublist in mpc.x_obs for item in sublist]
            # mpc.y_obs = [item for sublist in mpc.y_obs for item in sublist]

            mpc.initial_state = x1
            constraint1 = NonlinearConstraint(fun=mpc.propagation1, lb=-width_init/2, ub=width_init/2)
            constraint2 = NonlinearConstraint(fun=mpc.propagation2, lb=-height_init/2, ub=height_init/2)
            # if mpc.x_obs == [] and mpc.y_obs == []:
            #     constraints = [constraint1, constraint2]
            # else:
            #     constraint3 = NonlinearConstraint(fun=mpc.propagation3, lb=0, ub=np.inf)
            #     constraints = [constraint1, constraint2, constraint3]
            constraints = [constraint1, constraint2]
            
            # ob = []
            # for idx in range(robot_num):
            #     if idx == i:
            #         continue
            #     # ob.append([x[0, idx], x[1, idx]])
            #     ob.append([predicted_trajectory[i, :, 0].tolist(), predicted_trajectory[i, :, 1].tolist()])
            
            # mpc.x_obs = [ob[z][0] for z in range(len(ob))]
            # mpc.y_obs = [ob[z][1] for z in range(len(ob))]

            # MPC control
            print("Robot " + str(i))
            u_solution = minimize(mpc.cost_function3, u1, (x1, ref[i]),
                                method='SLSQP',
                                bounds=bounds,
                                constraints=constraints,
                                tol = 1e-3)
            
            if debug:
                print('Step ' + str(i) + ' of ' + str(iterations) + '   Time ' + str(round(time.time() - start_time,5)))
            u1 = u_solution.x
            x1 = mpc.plant_model(x1, dt, u1[0], u1[1])
            x[:, i] = x1
            u[:, i] = u1
            predicted_state = np.array([x1])

            for j in range(1, mpc.horizon):
                predicted = mpc.plant_model(predicted_state[-1], mpc.dt, u1[2*j], u1[2*j+1])
                predicted_state = np.append(predicted_state, np.array([predicted]), axis=0)
            predicted_trajectory[i, :, :] = predicted_state

            # plt.plot(mpc.x_obs, mpc.y_obs, "xk")
            utils.plot_robot(x1[0], x1[1], x1[2])
            # utils.plot_robot(ref[i][0],ref[i][1],ref[i][2])
            plt.plot(ref[i][0],ref[i][1], "xg")
            plt.plot(cx[i], cy[i], "-r", label="course")
            plt.plot(predicted_trajectory[i, :, 0], predicted_trajectory[i, :, 1], "-g")
            # plt.plot(predicted_state[:,0], predicted_state[:,1])
            
        plt.title('MPC 2D')
        utils.plot_map(width=width_init, height=height_init)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

def main5():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    iterations = 3000
    break_flag = False
    dl = 2
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    OBSTACLES = options['OBSTACLES']
    
    # x0, y, yaw, v, omega, model_type = utils.samplegrid(width_init, height_init, min_dist, robot_num, safety_init)
    # x = np.array([x0, y, yaw, v])
    x = np.array([[0, 10], [0, 0], [0, np.pi], [0,0]])

    ref = [[10, 0, 0], [0, 0, -np.pi]]

    # MPC initialization
    mpc = ModelPredictiveControl(obs_x=[], obs_y=[])
    # mpc = ModelPredictiveControl(obs_x=cx[0][5:-1:12], obs_y=cy[0][5:-1:12])

    num_inputs = 2
    u = np.zeros([mpc.horizon*num_inputs, robot_num])
    bounds = []

    # Set bounds for inputs bounded optimization.
    for i in range(mpc.horizon):
        bounds += [[min_acc, max_acc]]
        bounds += [[-max_steer, max_steer]]

    predicted_trajectory = np.zeros((robot_num, mpc.horizon, x.shape[0]))
    for i in range(robot_num):
        predicted_trajectory[i, :, :] = x[:, i] 
    
    # input [throttle, steer (delta)]
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)

    for z in range(iterations):
        # old_time = time.time()
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        for i in range(robot_num):
            x1 = x[:, i]
            # Updating the paths of the robots            
            u1 = u[:,i]
            u1 = np.delete(u1,0)
            u1 = np.delete(u1,0)
            u1 = np.append(u1, u1[-2])
            u1 = np.append(u1, u1[-2])
            start_time = time.time()


            mpc.x_obs = []
            mpc.y_obs = []
            for idx in range(robot_num):
                if idx == i:
                    continue
                # ob.append([x[0, idx], x[1, idx]])
                # mpc.x_obs.append(x[0, idx])
                # mpc.y_obs.append(x[1, idx])
                # ob.append([predicted_trajectory[i, :, 0].tolist(), predicted_trajectory[i, :, 1].tolist()])
                # add only if the robots are close enough

                # if dist([x1[0], x1[1]], [x[0, idx], x[1, idx]]) < 10:
                # if dist([x1[0], x1[1]], [x[0, idx], x[1, idx]]) < 1: raise Exception('Collision')
                mpc.x_obs.append(predicted_trajectory[idx, 0:-1:5, 0])
                mpc.y_obs.append(predicted_trajectory[idx, 0:-1:5, 1])
            mpc.x_obs = [item for sublist in mpc.x_obs for item in sublist]
            mpc.y_obs = [item for sublist in mpc.y_obs for item in sublist]

            mpc.initial_state = x1
            constraint1 = NonlinearConstraint(fun=mpc.propagation1, lb=-width_init/2, ub=width_init/2)
            constraint2 = NonlinearConstraint(fun=mpc.propagation2, lb=-height_init/2, ub=height_init/2)
            if mpc.x_obs == [] and mpc.y_obs == []:
                constraints = [constraint1, constraint2]
            else:
                constraint3 = NonlinearConstraint(fun=mpc.propagation3, lb=0, ub=np.inf)
                constraints = [constraint1, constraint2, constraint3]
            # constraints = [constraint1, constraint2]
            

            # MPC control
            # print("Robot " + str(i))
            u_solution = minimize(mpc.cost_function3, u1, (x1, ref[i]),
                                method='SLSQP',
                                bounds=bounds,
                                constraints=constraints,
                                tol = 1e-3)
            
            if debug:
                print('Step ' + str(i) + ' of ' + str(iterations) + '   Time ' + str(round(time.time() - start_time,5)))
            u1 = u_solution.x
            x1 = mpc.plant_model(x1, dt, u1[0], u1[1])
            x[:, i] = x1
            u[:, i] = u1
            predicted_state = np.array([x1])

            for j in range(1, mpc.horizon):
                predicted = mpc.plant_model(predicted_state[-1], mpc.dt, u1[2*j], u1[2*j+1])
                predicted_state = np.append(predicted_state, np.array([predicted]), axis=0)
            predicted_trajectory[i, :, :] = predicted_state

            # plt.plot(mpc.x_obs, mpc.y_obs, "xk")
            utils.plot_robot(x1[0], x1[1], x1[2])
            # utils.plot_robot(ref[i][0],ref[i][1],ref[i][2])
            plt.plot(ref[i][0],ref[i][1], "xg")
            plt.plot(predicted_trajectory[i, :, 0], predicted_trajectory[i, :, 1], "-g")
            # plt.plot(predicted_state[:,0], predicted_state[:,1])
            
        plt.title('MPC 2D')
        utils.plot_map(width=width_init, height=height_init)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

if __name__ == '__main__':
    # main()
    # main2()
    # main4()
    main5()