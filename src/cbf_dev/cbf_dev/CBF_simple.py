import numpy as np

from cvxopt import matrix, solvers
from cvxopt import matrix
from planner.utils import *

from custom_message.msg import ControlInputs, MultiControl

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
max_steer = json_object["CBF_simple"]["max_steer"]  # [rad] max steering angle
max_speed = json_object["Car_model"]["max_speed"] # [m/s]
min_speed = json_object["Car_model"]["min_speed"] # [m/s]
max_acc = json_object["CBF_simple"]["max_acc"] 
min_acc = json_object["CBF_simple"]["min_acc"] 
dt = json_object["CBF_simple"]["dt"]
safety_radius = json_object["CBF_simple"]["safety_radius"]
barrier_gain = json_object["CBF_simple"]["barrier_gain"]
arena_gain = json_object["CBF_simple"]["arena_gain"]
Kv = json_object["CBF_simple"]["Kv"] # interval [0.5-1]
Lr = L / 2.0  # [m]
Lf = L - Lr
robot_num = json_object["robot_num"]
safety = json_object["safety"]
width = json_object["width"]
height = json_object["height"]
boundary_points = np.array([-width/2, width/2, -height/2, height/2])

def CBF(x, u_ref):
    """
    Computes the control input for the C3BF (Collision Cone Control Barrier Function) algorithm.

    Args:
        x (numpy.ndarray): State vector of shape (4, N), where N is the number of time steps.
        u_ref (numpy.ndarray): Reference control input of shape (2, N).

    Returns:
        numpy.ndarray: Filtered Control input dxu of shape (2, N).

    """
    N = x.shape[1]
    M = u_ref.shape[0]
    dxu = np.zeros([u_ref.shape[0], u_ref.shape[1]])

    u_ref[1,:] = delta_to_beta_array(u_ref[1,:])

    for i in range(N):
        count = 0
        G = np.zeros([N-1,M])
        H = np.zeros([N-1,1])

        # when the car goes backwards the yaw angle should be flipped --> Why??
        # x[2,i] = (1-np.sign(x[3,i]))*(np.pi/2) + x[2,i]

        f = np.array([x[3,i]*np.cos(x[2,i]),
                          x[3,i]*np.sin(x[2,i]), 
                          0, 
                          0]).reshape(4,1)
        g = np.array([[0, -x[3,i]*np.sin(x[2,i])], 
                        [0, x[3,i]*np.cos(x[2,i])], 
                        [0, x[3,i]/Lr],
                        [1, 0]]).reshape(4,2)

        for j in range(N):

            if j == i: continue

            P = np.identity(2)*2
            q = np.array([-2 * u_ref[0, i], - 2 * u_ref[1,i]])

            Lf_h = 2 * x[3,i] * (np.cos(x[2,i]) * (x[0,i]-x[0,j]) + np.sin(x[2,i]) * (x[1,i] - x[1,j]))
            Lg_h = 2 * x[3,i] * (np.cos(x[2,i]) * (x[1,i]-x[1,j]) - np.sin(x[2,i]) * (x[0,i] - x[0,j]))
            h = (x[0,i]-x[0,j]) * (x[0,i]-x[0,j]) + (x[1,i] - x[1,j]) * (x[1,i] - x[1,j]) - (safety_radius**2 + Kv * abs(x[3,i]))

            H[count] = np.array([barrier_gain*np.power(h, 3) + Lf_h])

            if x[3,i] >= 0:
                G[count,:] = np.array([Kv, -Lg_h])
            else:
                G[count,:] = np.array([-Kv, -Lg_h])
                
            count+=1
        
        # Add the input constraint
        G = np.vstack([G, [[0, 1], [0, -1]]])
        H = np.vstack([H, delta_to_beta(max_steer), -delta_to_beta(-max_steer)])
        G = np.vstack([G, [[1, 0], [-1, 0]]])
        H = np.vstack([H, max_acc, -min_acc])

        # Adding arena boundary constraints
        # Pos Y
        h = ((x[1,i] - boundary_points[3])**2 - safety_radius**2 - Kv * abs(x[3,i]))
        if x[3,i] >= 0:
            gradH = np.array([0, 2*(x[1,i] - boundary_points[3]), 0, -Kv])
        else:
            gradH = np.array([0, 2*(x[1,i] - boundary_points[3]), 0, Kv])

        Lf_h = np.dot(gradH.T, f)
        Lg_h = np.dot(gradH.T, g)
        G = np.vstack([G, -Lg_h])
        H = np.vstack([H, np.array([arena_gain*h**3 + Lf_h])])

        # Neg Y
        h = ((x[1,i] - boundary_points[2])**2 - safety_radius**2 - Kv * abs(x[3,i]))
        if x[3,i] >= 0:
            gradH = np.array([0, 2*(x[1,i] - boundary_points[2]), 0, -Kv])
        else:
            gradH = np.array([0, 2*(x[1,i] - boundary_points[2]), 0, Kv])
        Lf_h = np.dot(gradH.T, f)
        Lg_h = np.dot(gradH.T, g)
        G = np.vstack([G, -Lg_h])
        H = np.vstack([H, np.array([arena_gain*h**3 + Lf_h])])

        # Pos X
        h = ((x[0,i] - boundary_points[1])**2 - safety_radius**2 - Kv * abs(x[3,i]))
        if x[3,i] >= 0:
            gradH = np.array([2*(x[0,i] - boundary_points[1]), 0, 0, -Kv])
        else:
            gradH = np.array([2*(x[0,i] - boundary_points[1]), 0, 0, Kv])
        Lf_h = np.dot(gradH.T, f)
        Lg_h = np.dot(gradH.T, g)
        G = np.vstack([G, -Lg_h])
        H = np.vstack([H, np.array([arena_gain*h**3 + Lf_h])])

        # Neg X
        h = ((x[0,i] - boundary_points[0])**2 - safety_radius**2 - Kv * abs(x[3,i]))
        if x[3,i] >= 0:
            gradH = np.array([2*(x[0,i] - boundary_points[0]), 0, 0, -Kv])
        else:
            gradH = np.array([2*(x[0,i] - boundary_points[0]), 0, 0, Kv])
        Lf_h = np.dot(gradH.T, f)
        Lg_h = np.dot(gradH.T, g)
        G = np.vstack([G, -Lg_h])
        H = np.vstack([H, np.array([arena_gain*h**3 + Lf_h])])
        
        solvers.options['show_progress'] = False
        sol = solvers.qp(matrix(P), matrix(q), matrix(G), matrix(H))
        dxu[:,i] = np.reshape(np.array(sol['x']), (M,))
    
    dxu[1,:] = beta_to_delta(dxu[1,:])
    return dxu

def delta_to_beta(delta):
    """
    Converts the steering angle delta to the slip angle beta.

    Args:
        delta (float): Steering angle in radians.

    Returns:
        float: Slip angle in radians.

    """
    beta = normalize_angle(np.arctan2(Lr*np.tan(delta)/L, 1.0))

    return beta

def delta_to_beta_array(delta):
    """
    Converts an array of steering angles delta to an array of slip angles beta.

    Args:
        delta (numpy.ndarray): Array of steering angles in radians.

    Returns:
        numpy.ndarray: Array of slip angles in radians.

    """
    beta = normalize_angle_array(np.arctan2(Lr*np.tan(delta)/L, 1.0))

    return beta

def beta_to_delta(beta):
    """
    Converts the slip angle beta to the steering angle delta.

    Args:
        beta (float): Slip angle in radians.

    Returns:
        float: Steering angle in radians.

    """
    delta = normalize_angle_array(np.arctan2(L*np.tan(beta)/Lr, 1.0))

    return delta

def plot_rect(x, y, yaw, r):  # pragma: no cover
        """
    Plots a rectangle with the given parameters.

    Args:
        x (float): x-coordinate of the center of the rectangle.
        y (float): y-coordinate of the center of the rectangle.
        yaw (float): Orientation of the rectangle in radians.
        r (float): Length of the sides of the rectangle.

    """
        outline = np.array([[-r / 2, r / 2,
                                (r / 2), -r / 2,
                                -r / 2],
                            [r / 2, r/ 2,
                                - r / 2, -r / 2,
                                r / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                            [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), "-k")

def update_paths(paths):
    """
    Updates the given paths.

    Args:
        paths (list): List of paths.

    Returns:
        list: Updated paths.

    """
    updated_paths = []
    for path in paths:
        updated_paths.append(update_path(path))
    return updated_paths

def check_collision(x,i):
    """
    Checks for collision between the robot at index i and other robots.

    Args:
        x (numpy.ndarray): State vector of shape (4, N), where N is the number of time steps.
        i (int): Index of the robot to check collision for.

    Raises:
        Exception: If collision is detected.

    """
    for idx in range(robot_num):
        if idx == i:
            continue
        if dist([x[0,i], x[1,i]], [x[0, idx], x[1, idx]]) < WB:
            raise Exception('Collision')

def plot_robot_and_arrows(i, x, multi_control, targets):
    """
    Plots the robot and arrows for visualization.

    Args:
        i (int): Index of the robot.
        x (numpy.ndarray): State vector of shape (4, N), where N is the number of time steps.
        multi_control (numpy.ndarray): Control inputs of shape (2, N).
        targets (list): List of target points.

    """
    plot_robot(x[0, i], x[1, i], x[2, i])
    plot_arrow(x[0, i], x[1, i], x[2, i] + multi_control.multi_control[i].delta, length=3, width=0.5)
    plot_arrow(x[0, i], x[1, i], x[2, i], length=1, width=0.5)
    plt.plot(targets[i][0], targets[i][1], "xg")

def control_robot(x, targets, robot_num, multi_control):
    """
    Controls the movement of a robot based on its current state and target positions.

    Args:
        i (int): Index of the robot.
        x (numpy.ndarray): Array representing the current state of all robots.
        targets (list): List of target positions for each robot.
        robot_num (int): Total number of robots.
        multi_control (MultiControl): Object for storing control inputs for all robots.
        paths (list): List of paths for each robot.

    Returns:
        tuple: Updated state of all robots, updated target positions, and updated multi_control object.
    """
    dxu = np.zeros((2, robot_num))

    cmd = ControlInputs()
    
    for idx in range(robot_num):
        check_collision(x, idx)
        x1 = array_to_state(x[:, idx])
        cmd.throttle, cmd.delta = pure_pursuit_steer_control(targets[idx], x1)
        dxu[0, idx], dxu[1, idx] = cmd.throttle, cmd.delta

    dxu = CBF(x, dxu)

    for idx in range(robot_num):
        x1 = array_to_state(x[:, idx])
        cmd.throttle, cmd.delta = dxu[0, idx], dxu[1, idx]
        x1 = linear_model_callback(x1, cmd)
        x1 = state_to_array(x1).reshape(4)
        x[:, idx] = x1
        multi_control.multi_control[idx] = cmd

        plot_robot_and_arrows(idx, x, multi_control, targets)

    return x, targets, multi_control

def main(args=None):
    """
    Main function for controlling multiple robots using Model Predictive Control (MPC).

    Steps:
    1. Initialize the necessary variables and parameters.
    2. Create an instance of the ModelPredictiveControl class.
    3. Set the initial state and control inputs.
    4. Generate the reference trajectory for each robot.
    5. Plot the initial positions and reference trajectory.
    6. Set the bounds and constraints for the MPC.
    7. Initialize the predicted trajectory for each robot.
    8. Enter the main control loop:
        - Check if the distance between the current position and the target is less than 5.
            - If yes, update the path and target.
        - Perform CBF control for each robot.
        - Plot the robot trajectory.
        - Update the predicted trajectory.
        - Plot the map and pause for visualization.
    """
    # Step 1: Set the number of iterations
    iterations = 3000
    
    # Step 2: Sample initial values for x0, y, yaw, v, omega, and model_type
    x0, y, yaw, v, omega, model_type = samplegrid(width_init, height_init, min_dist, robot_num, safety_init)
    
    # Step 3: Create an array x with the initial values
    x = np.array([x0, y, yaw, v])
    
    # Step 4: Create paths for each robot
    paths = [create_path() for _ in range(robot_num)]
    
    # Step 5: Extract the target coordinates from the paths
    targets = [[path[0].x, path[0].y] for path in paths]
    
    # Step 6: Create a MultiControl object
    multi_control = MultiControl()
    
    # Step 7: Initialize the multi_control list with ControlInputs objects
    multi_control.multi_control = [ControlInputs(delta=0.0, throttle=0.0) for _ in range(robot_num)]
    
    # Step 8: Perform the simulation for the specified number of iterations
    for z in range(iterations):
        plt.cla()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        for i in range(robot_num):
            # Step 9: Check if the distance between the current position and the target is less than 5
            if dist(point1=(x[0,i], x[1,i]), point2=targets[i]) < 5:
                # Perform some action when the condition is met
                pass
                paths[i] = update_path(paths[i])
                targets[i] = (paths[i][0].x, paths[i][0].y)

        x, targets, multi_control = control_robot(x, targets, robot_num, multi_control)
        
        plot_map(width=width_init, height=height_init)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)


if __name__=='__main__':
    main()
        
