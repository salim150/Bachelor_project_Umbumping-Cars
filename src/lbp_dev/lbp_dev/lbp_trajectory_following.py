import json
import numpy as np
import pathlib
import math
from LBP import normalize_angle
from shapely.geometry import Point, Polygon, LineString
import matplotlib.pyplot as plt

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
dt = json_object["Controller"]["dt"] # [s] Time tick for motion prediction
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
v_ref = 2.0 # [m/s] reference speed

# Load trajectory and control inputs from LBP.json file
with open('src/lbp_dev/lbp_dev/LBP.json', 'r') as file:
    data = json.load(file)

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

def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_robot(x, y, yaw):  # pragma: no cover
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


# Implement kinematic bicycle model to follow the trajectory
def main():
    v = 2.0 # [m/s] reference speed
    x = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v]   
    ref = data[str(v)]['1']
    trajectory = np.zeros((len(ref['x']),3))
    trajectory[:,0] = ref['x']
    trajectory[:,1] = ref['y']
    trajectory[:,2] = ref['yaw']
    control_inputs = ref['ctrl']

    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(111)

    for i in range(len(control_inputs)):
        # Get current state and control input
        delta = control_inputs[i]
        a = (v-x[3])/dt
        u = [a, delta]

        # Apply control input to the kinematic bicycle model
        # TODO: Implement the kinematic bicycle model logic here
        x = motion(x, u, dt)

        # Print the current state and control input
        print(f"Control Input: {u}")
        print(f'Current State: {x[0:3]}, reference: {trajectory[i,:]}')
        print(f'Current Speed: {x[3]}\n')
        
        # plot the trajectory, the robot and the yaw
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-g")
        # plot_polygon(dilated_traj[i], ax=ax, add_points=False, alpha=0.5)
        plt.plot(x[0], x[1], "xr")
        plt.plot(trajectory[-1, 0], trajectory[-1, 1], "xb")
        plot_robot(x[0], x[1], x[2])
        plot_arrow(x[0], x[1], x[2], length=1, width=0.2)
        plot_arrow(x[0], x[1], x[2]+u[1], length=1, width=0.2)
  
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)



if __name__ == "__main__":
    main()