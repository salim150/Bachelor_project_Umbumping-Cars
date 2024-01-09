import itertools
import numpy as np
from scipy.special import comb

from cvxopt import matrix, solvers
from cvxopt.blas import dot
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse
from planner.utils import *
from planner.utils import plot_robot
from planner.predict_traj import predict_trajectory

from custom_message.msg import ControlInputs, State, Path, Coordinate, MultiplePaths, MultiState, MultiControl

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
    Computes the circular Control Barrier Function (CBF) for a given state and reference input.

    Args:
        x (numpy.ndarray): State vector.
        u_ref (numpy.ndarray): Reference input vector.

    Returns:
        numpy.ndarray: Computed control input.

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
        dxu[:,count_dxu] = np.reshape(np.array(sol['x']), (M,))
        count_dxu += 1
    
    dxu[1,:] = beta_to_delta(dxu[1,:])
    return dxu

def delta_to_beta(delta):
    beta = normalize_angle(np.arctan2(Lr*np.tan(delta)/L, 1.0))

    return beta

def delta_to_beta_array(delta):
    beta = normalize_angle_array(np.arctan2(Lr*np.tan(delta)/L, 1.0))

    return beta

def beta_to_delta(beta):
    delta = normalize_angle_array(np.arctan2(L*np.tan(beta)/Lr, 1.0))

    return delta

def plot_rect(x, y, yaw, r):  # pragma: no cover
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


def main(args=None):
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
            if dist(point1=(x[0,i], x[1,i]), point2=targets[i]) < 10:
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
            dxu = CBF(x, dxu)
            cmd.throttle, cmd.delta = dxu[0,i], dxu[1,i]
            x1 = linear_model_callback(x1, cmd)
            x1 = state_to_array(x1).reshape(4)
            x[:, i] = x1
            multi_control.multi_control[i] = cmd
    
            # plt.plot(x[0,i], x[1,i], "xr")
            # plt.plot(goal[0,i], goal[1,i], "xb")plot_arrow(x1.x, x1.y, x1.yaw)
            plot_robot(x[0,i], x[1,i], x[2,i])
            # plot_rect(x[0,i], x[1,i], x[2,i], safety_radius)
            plot_arrow(x[0,i], x[1,i], x[2,i] + multi_control.multi_control[i].delta, length=2, width=1)
            plot_arrow(x[0,i], x[1,i], x[2,i], length=2, width=1)
            plt.plot(targets[i][0], targets[i][1], "xg")
            # plot_path(multi_traj.multiple_path[i])

        plot_map(width=width_init, height=height_init)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)


if __name__=='__main__':
    main()
        
