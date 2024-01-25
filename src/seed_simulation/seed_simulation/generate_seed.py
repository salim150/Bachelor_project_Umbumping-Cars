import numpy as np
import os
from cvxopt import matrix, solvers
from cvxopt import matrix
from planner.utils import *
# import planner.utils as utils

from custom_message.msg import ControlInputs, State, MultiControl

# For the parameter file
import pathlib
import json

# TODO: import all this parameters from a config file so that we can easily change them in one place
path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

L = json_object["Car_model"]["L"]
max_steer = json_object["C3BF"]["max_steer"]  # [rad] max steering angle
max_speed = json_object["Car_model"]["max_speed"] # [m/s]
min_speed = json_object["Car_model"]["min_speed"] # [m/s]
max_acc = json_object["C3BF"]["max_acc"] 
min_acc = json_object["C3BF"]["min_acc"] 
dt = json_object["C3BF"]["dt"]
safety_radius = json_object["C3BF"]["safety_radius"]
barrier_gain = json_object["C3BF"]["barrier_gain"]
arena_gain = json_object["C3BF"]["arena_gain"]
Kv = json_object["C3BF"]["Kv"] # interval [0.5-1]
Lr = L / 2.0  # [m]
Lf = L - Lr
robot_num = json_object["robot_num"]
safety_init = json_object["safety"]
width_init = json_object["width"]
height_init = json_object["height"]
min_dist = json_object["min_dist"]
boundary_points = np.array([-width_init/2, width_init/2, -height_init/2, height_init/2])

# write a main function that generates a path for robot_num robots using the function create_path and saves the the generated trajectories to a dictionary.
# The dictionary is then saved to a file using the function save_dict_to_file
def save_dict_to_file(dict):
    # save the dictionary to a file
    filename = 'src/seed_'
    i = 0
    while os.path.exists(f"{filename}{i}.json"):
        i += 1
    print(f"Saving seed to {filename}{i}.json")
    with open(f'{filename}{i}.json', 'w') as fp:
        json.dump(dict, fp, indent=3)

def main():
    # generate a path for robot_num robots
    # save the generated trajectories to a dictionary
    # save the dictionary to a file
    # create a dictionary to save the generated trajectories
    initial_position = {}
    x0, y, yaw, v, omega, model_type = samplegrid(width_init, height_init, min_dist, robot_num, safety_init)
    
    initial_position['x'] = x0
    initial_position['y'] = y
    initial_position['yaw'] = yaw
    initial_position['v'] = v

    trajectories = {}
    for i in range(robot_num):
        # generate a path for each robot
        trajectories[i] = create_seed(len_path=10)
    # save the dictionary to a file
        
    seed = {}
    seed['initial_position'] = initial_position
    seed['trajectories'] = trajectories
    save_dict_to_file(seed)

if __name__ == "__main__":
    main()