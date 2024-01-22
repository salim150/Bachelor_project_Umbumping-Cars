import numpy as np

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

def motion(x, u, dt):
    """
    Motion model for a robot.

    Args:
        x (list): Initial state of the robot [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)].
        u (list): Control inputs [throttle, delta].
        dt (float): Time step.

    Returns:
        list: Updated state of the robot.

    """
    delta = u[1]
    delta = np.clip(delta, -max_steer, max_steer)
    throttle = u[0]

    x[0] = x[0] + x[3] * math.cos(x[2]) * dt
    x[1] = x[1] + x[3] * math.sin(x[2]) * dt
    x[2] = x[2] + x[3] / L * math.tan(delta) * dt
    x[2] = normalize_angle(x[2])
    x[3] = x[3] + throttle * dt
    x[3] = np.clip(x[3], min_speed, max_speed)

    return x

def C3BF(x, u_ref):
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
    count_dxu = 0

    u_ref[1,:] = delta_to_beta_array(u_ref[1,:])

    for i in range(N):
        count = 0
        G = np.zeros([N-1,M])
        H = np.zeros([N-1,1])

        # when the car goes backwards the yaw angle should be flipped --> Why??
        x[2,i] = (1-np.sign(x[3,i]))*(np.pi/2) + x[2,i]

        f = np.array([x[3,i]*np.cos(x[2,i]),
                          x[3,i]*np.sin(x[2,i]), 
                          0, 
                          0]).reshape(4,1)
        g = np.array([[0, -x[3,i]*np.sin(x[2,i])], 
                        [0, x[3,i]*np.cos(x[2,i])], 
                        [0, x[3,i]/Lr],
                        [1, 0]]).reshape(4,2)
        
        P = np.identity(2)*2
        q = np.array([-2 * u_ref[0, i], - 2 * u_ref[1,i]])
        
        for j in range(N):
            arr = np.array([x[0, j] - x[0, i], x[1, j] - x[1,i]])
            dist = np.linalg.norm(arr)
            v = np.array([x[3,i]*np.cos(x[2,i]), x[3,i]*np.sin(x[2,i])])
            scalar_prod = v @ arr

            if j == i or dist > 3 * safety_radius or scalar_prod < 0: 
                continue

            v_rel = np.array([x[3,j]*np.cos(x[2,j]) - x[3,i]*np.cos(x[2,i]), 
                              x[3,j]*np.sin(x[2,j]) - x[3,i]*np.sin(x[2,i])])
            p_rel = np.array([x[0,j]-x[0,i],
                              x[1,j]-x[1,i]])
            
            cos_Phi = np.sqrt(abs(np.linalg.norm(p_rel)**2 - safety_radius**2))/np.linalg.norm(p_rel)
            tan_Phi_sq = safety_radius**2 / (np.linalg.norm(p_rel)**2 - safety_radius**2)
            
            h = np.dot(p_rel, v_rel) + np.linalg.norm(v_rel) * np.linalg.norm(p_rel) * cos_Phi
            
            gradH_1 = np.array([- (x[3,j]*np.cos(x[2,j]) - x[3,i]*np.cos(x[2,i])), 
                                - (x[3,j]*np.sin(x[2,j]) - x[3,i]*np.sin(x[2,i])),
                                x[3,i] * (np.sin(x[2,i]) * p_rel[0] - np.cos(x[2,i]) * p_rel[1]),
                                -np.cos(x[2,i]) * p_rel[0] - np.sin(x[2,i]) * p_rel[1]])
            
            gradH_21 = -(1 + tan_Phi_sq) * np.linalg.norm(v_rel)/np.linalg.norm(p_rel) * cos_Phi * p_rel 
            gradH_22 = np.dot(np.array([x[3,i]*np.sin(x[2,i]), -x[3,i]*np.cos(x[2,i])]), v_rel) * np.linalg.norm(p_rel)/(np.linalg.norm(v_rel) + 0.00001) * cos_Phi
            gradH_23 = - np.dot(v_rel, np.array([np.cos(x[2,i]), np.sin(x[2,i])])) * np.linalg.norm(p_rel)/(np.linalg.norm(v_rel) + 0.00001) * cos_Phi

            gradH = gradH_1.reshape(4,1) + np.vstack([gradH_21.reshape(2,1), gradH_22, gradH_23])

            Lf_h = np.dot(gradH.T, f)
            Lg_h = np.dot(gradH.T, g)

            H[count] = np.array([barrier_gain*np.power(h, 1) + Lf_h])
            G[count,:] = -Lg_h
            count+=1

        # Adding arena boundary constraints TODO check the calculation/propagation of Kv in the lie derivatives
        # Pos Y
        # h = 0.1*(boundary_points[3] - safety_radius - x[1,i] - Kv * x[3,i])**3
        # gradH = np.array([0,-1, 0, -Kv])
        # Lf_h = np.dot(gradH.T, f)
        # Lg_h = np.dot(gradH.T, g)
        # G = np.vstack([G, -Lg_h])
        # H = np.vstack([H, np.array([h + Lf_h])])

        # # Neg Y
        # h = 0.1*(-boundary_points[2] - safety_radius + x[1,i] - Kv * x[3,i])**3
        # gradH = np.array([0,1, x[3,i]*np.cos(x[2,i]), np.sin(x[2,i])])
        # Lf_h = np.dot(gradH.T, f)
        # Lg_h = np.dot(gradH.T, g)
        # G = np.vstack([G, -Lg_h])
        # H = np.vstack([H, np.array([h + Lf_h])])

        # # Pos X
        # h = 0.1*(boundary_points[1] - safety_radius - x[0,i] - Kv * x[3,i])**3
        # gradH = np.array([-1,0, x[3,i]*np.sin(x[2,i]), -np.cos(x[2,i])])
        # Lf_h = np.dot(gradH.T, f)
        # Lg_h = np.dot(gradH.T, g)
        # G = np.vstack([G, -Lg_h])
        # H = np.vstack([H, np.array([h + Lf_h])])

        # # Neg X
        # h = 0.1*(-boundary_points[0] - safety_radius + x[0,i] - Kv * x[3,i])**3
        # gradH = np.array([1,0, -x[3,i]*np.sin(x[2,i]), np.cos(x[2,i])])
        # Lf_h = np.dot(gradH.T, f)
        # Lg_h = np.dot(gradH.T, g)
        # G = np.vstack([G, -Lg_h])
        # H = np.vstack([H, np.array([h + Lf_h])])

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
        
        # Input constraints
        G = np.vstack([G, [[0, 1], [0, -1]]])
        H = np.vstack([H, delta_to_beta(max_steer), -delta_to_beta(-max_steer)])
        # G = np.vstack([G, [[1, 0], [-1, 0]]])
        # H = np.vstack([H, max_acc, -min_acc])

        solvers.options['show_progress'] = False
        sol = solvers.qp(matrix(P), matrix(q), matrix(G), matrix(H))
        dxu[:,count_dxu] = np.reshape(np.array(sol['x']), (M,))
        count_dxu += 1
    
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

def control_robot(i, x, targets, robot_num, multi_control):
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

    check_collision(x, i)

    x1 = array_to_state(x[:, i])
    cmd = ControlInputs()
    cmd.throttle, cmd.delta = pure_pursuit_steer_control(targets[i], x1)
    dxu[0, i], dxu[1, i] = cmd.throttle, cmd.delta

    dxu = C3BF(x, dxu)

    cmd.throttle, cmd.delta = dxu[0, i], dxu[1, i]
    x1 = linear_model_callback(x1, cmd)
    x1 = state_to_array(x1).reshape(4)
    x[:, i] = x1
    multi_control.multi_control[i] = cmd

    plot_robot_and_arrows(i, x, multi_control, targets)

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
        - Perform 3CBF control for each robot.
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

            x, targets, multi_control = control_robot(i, x, targets, robot_num, multi_control)
        
        plot_map(width=width_init, height=height_init)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

def main1(args=None):
    # Instantiate Robotarium object
    # The robots will never reach their goal points so set iteration number
    iterations = 3000
    # Define goal points outside of the arena
    x0, y, yaw, v, omega, model_type = samplegrid(width_init, height_init, min_dist, robot_num, safety_init)
    x = np.array([x0, y, yaw, v])

    paths = []
    targets = []
    # multi_traj = MultiplePaths()
    multi_control = MultiControl()
    dxu = np.zeros((2,robot_num))
    for i in range(robot_num):
        paths.append(create_path())
        targets.append([paths[i][0].x, paths[i][0].y])
        initial_state = State(x=x0[i], y=y[i], yaw=yaw[i], v=v[i], omega=omega[i])
        # multi_traj.multiple_path.append(predict_trajectory(initial_state, targets[i]))
        multi_control.multi_control.append(ControlInputs(delta=0.0, throttle=0.0))
        
    # While the number of robots at the required poses is less
    # than N...
    for z in range(iterations):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        # Create single-integrator control inputs
        for i in range(robot_num):
            x1 = array_to_state(x[:,i])
            if dist(point1=(x[0,i], x[1,i]), point2=targets[i]) < 5:
                paths[i] = update_path(paths[i])
                targets[i] = (paths[i][0].x, paths[i][0].y)
                # multi_traj.multiple_path[i] = predict_trajectory(x1, targets[i])

            for idx in range(robot_num):
                if idx == i:
                    continue
                if dist([x1.x, x1.y], [x[0, idx], x[1, idx]]) < WB: raise Exception('Collision')
            
            cmd = ControlInputs()
            cmd.throttle, cmd.delta= pure_pursuit_steer_control(targets[i], x1)

            dxu[0,i], dxu[1,i] = cmd.throttle, cmd.delta            
            dxu = C3BF(x, dxu)
            cmd.throttle, cmd.delta = dxu[0,i], dxu[1,i]
            x1 = linear_model_callback(x1, cmd)
            x1 = state_to_array(x1).reshape(4)
            x[:, i] = x1
            multi_control.multi_control[i] = cmd
    
            # plt.plot(x[0,i], x[1,i], "xr")
            # plt.plot(goal[0,i], goal[1,i], "xb")plot_arrow(x1.x, x1.y, x1.yaw)
            plot_robot(x[0,i], x[1,i], x[2,i])
            # plot_rect(x[0,i], x[1,i], x[2,i], safety_radius)
            plot_arrow(x[0,i], x[1,i], x[2,i] + multi_control.multi_control[i].delta, length=3, width=0.5)
            plot_arrow(x[0,i], x[1,i], x[2,i], length=1, width=0.5)
            plt.plot(targets[i][0], targets[i][1], "xg")
            # plot_path(multi_traj.multiple_path[i])

        plot_map(width=width_init, height=height_init)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)


    
if __name__=='__main__':
    main()
        
