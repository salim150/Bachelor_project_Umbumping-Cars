import itertools
import numpy as np
from scipy.special import comb

from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse
from planner.utils import *
from planner.predict_traj import predict_trajectory

# TODO: import all this parameters from a config file so that we can easily change them in one place
L = 2.9
max_steer = np.radians(30.0)  # [rad] max steering angle
max_speed = 6 # [m/s]
min_speed = -1.0 # [m/s]
magnitude_limit= max_speed
dt = 0.1
safety_radius = 7
barrier_gain = 0.05
magnitude_limit = max_speed

def create_unicycle_barrier_certificate_with_boundary(barrier_gain=barrier_gain, safety_radius=safety_radius, projection_distance=0.05, magnitude_limit=magnitude_limit, boundary_points = np.array([-50, 50, -50, 50])):
    """ Creates a unicycle barrier cetifcate to avoid collisions. Uses the diffeomorphism mapping
    and single integrator implementation. For optimization purposes, this function returns 
    another function.

    barrier_gain: double (how fast the robots can approach each other)
    safety_radius: double (how far apart the robots should stay)
    projection_distance: double (how far ahead to place the bubble)

    -> function (the unicycle barrier certificate function)
    """

    #Check user input types
    assert isinstance(barrier_gain, (int, float)), "In the function create_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be an integer or float. Recieved type %r." % type(barrier_gain).__name__
    assert isinstance(safety_radius, (int, float)), "In the function create_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(safety_radius).__name__
    assert isinstance(projection_distance, (int, float)), "In the function create_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    assert isinstance(magnitude_limit, (int, float)), "In the function create_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(magnitude_limit).__name__

    #Check user input ranges/sizes
    assert barrier_gain > 0, "In the function create_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be positive. Recieved %r." % barrier_gain
    assert safety_radius >= 0.12, "In the function create_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m). Recieved %r." % safety_radius
    assert projection_distance > 0, "In the function create_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be positive. Recieved %r." % projection_distance
    assert magnitude_limit > 0, "In the function create_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= max_speed, "In the function create_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit


    si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary(barrier_gain=barrier_gain, safety_radius=safety_radius+projection_distance, boundary_points=boundary_points)

    si_to_bi_dyn, bi_to_si_states = create_si_to_bi_mapping(projection_distance=projection_distance)

    bi_to_si_dyn = create_bi_to_si_dynamics(projection_distance=projection_distance)

    def f(dxu, x):
        #Check user input types
        assert isinstance(dxu, np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the unicycle robot velocity command (dxu) must be a numpy array. Recieved type %r." % type(dxu).__name__
        assert isinstance(x, np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(x).__name__

        #Check user input ranges/sizes
        assert x.shape[0] == 4, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the unicycle robot states (x) must be 3 ([x;y;theta]). Recieved dimension %r." % x.shape[0]
        assert dxu.shape[0] == 2, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the robot unicycle velocity command (dxu) must be 2 ([v;w]). Recieved dimension %r." % dxu.shape[0]
        assert x.shape[1] == dxu.shape[1], "In the function created by the create_unicycle_barrier_certificate function, the number of robot states (x) must be equal to the number of robot unicycle velocity commands (dxu). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (x.shape[0], x.shape[1], dxu.shape[0], dxu.shape[1])


        x_si = bi_to_si_states(x)
        #Convert unicycle control command to single integrator one
        dxi = bi_to_si_dyn(dxu, x)

        #Apply single integrator barrier certificate
        dxi = si_barrier_cert(dxi, x_si)

        #Return safe unicycle command
        return si_to_bi_dyn(dxi, x)

    return f

def create_single_integrator_barrier_certificate_with_boundary(barrier_gain=barrier_gain, safety_radius=safety_radius, magnitude_limit=magnitude_limit, boundary_points = np.array([-50, 50, -50, 50])):
    """Creates a barrier certificate for a single-integrator system with a rectangular boundary included.  This function
    returns another function for optimization reasons.

    barrier_gain: double (controls how quickly agents can approach each other.  lower = slower)
    safety_radius: double (how far apart the agents will stay)
    magnitude_limit: how fast the robot can move linearly.

    -> function (the barrier certificate function)
    """

    #Check user input types
    assert isinstance(barrier_gain, (int, float)), "In the function create_single_integrator_barrier_certificate, the barrier gain (barrier_gain) must be an integer or float. Recieved type %r." % type(barrier_gain).__name__
    assert isinstance(safety_radius, (int, float)), "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(safety_radius).__name__
    assert isinstance(magnitude_limit, (int, float)), "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(magnitude_limit).__name__

    #Check user input ranges/sizes
    assert barrier_gain > 0, "In the function create_single_integrator_barrier_certificate, the barrier gain (barrier_gain) must be positive. Recieved %r." % barrier_gain
    assert safety_radius >= 0.12, "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m) plus the distance to the look ahead point used in the diffeomorphism if that is being used. Recieved %r." % safety_radius
    assert magnitude_limit > 0, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= max_speed, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit
    assert magnitude_limit <= max_speed, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit


    def f(dxi, x):
        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the single-integrator robot velocity command (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(x, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(x).__name__

        #Check user input ranges/sizes
        assert x.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the single integrator robot states (x) must be 2 ([x;y]). Recieved dimension %r." % x.shape[0]
        assert dxi.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the robot single integrator velocity command (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert x.shape[1] == dxi.shape[1], "In the function created by the create_single_integrator_barrier_certificate function, the number of robot states (x) must be equal to the number of robot single integrator velocity commands (dxi). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (x.shape[0], x.shape[1], dxi.shape[0], dxi.shape[1])

        
        # Initialize some variables for computational savings
        N = dxi.shape[1]
        num_constraints = int(comb(N, 2)) + 4*N
        A = np.zeros((num_constraints, 2*N))
        b = np.zeros(num_constraints)
        #H = sparse(matrix(2*np.identity(2*N)))
        H = 2*np.identity(2*N)

        count = 0
        for i in range(N-1):
            for j in range(i+1, N):
                error = x[:, i] - x[:, j]
                h = (error[0]*error[0] + error[1]*error[1]) - np.power(safety_radius, 2)

                A[count, (2*i, (2*i+1))] = -2*error
                A[count, (2*j, (2*j+1))] = 2*error
                b[count] = barrier_gain*np.power(h, 3)

                count += 1
        
        for k in range(N):
            #Pos Y
            A[count, (2*k, 2*k+1)] = np.array([0,1])
            b[count] = 0.4*barrier_gain*(boundary_points[3] - safety_radius/2 - x[1,k])**3;
            count += 1

            #Neg Y
            A[count, (2*k, 2*k+1)] = -np.array([0,1])
            b[count] = 0.4*barrier_gain*(-boundary_points[2] - safety_radius/2 + x[1,k])**3;
            count += 1

            #Pos X
            A[count, (2*k, 2*k+1)] = np.array([1,0])
            b[count] = 0.4*barrier_gain*(boundary_points[1] - safety_radius/2 - x[0,k])**3;
            count += 1

            #Neg X
            A[count, (2*k, 2*k+1)] = -np.array([1,0])
            b[count] = 0.4*barrier_gain*(-boundary_points[0] - safety_radius/2 + x[0,k])**3;
            count += 1
        
        # Threshold control inputs before QP
        norms = np.linalg.norm(dxi, 2, 0)
        idxs_to_normalize = (norms > magnitude_limit)
        dxi[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]

        f = -2*np.reshape(dxi, (2*N,1), order='F')
        b = np.reshape(b, (count,1), order='F')
        result = qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']
        #result = solver2.solve_qp(H, f, A, b, 0)[0]

        return np.reshape(result, (2, N), order='F')

    return f

def create_si_to_bi_mapping(projection_distance=0.05, angular_velocity_limit = np.pi):
    """Creates two functions for mapping from single integrator dynamics to 
    unicycle dynamics and unicycle states to single integrator states. 
    
    This mapping is done by placing a virtual control "point" in front of 
    the unicycle.

    projection_distance: How far ahead to place the point
    angular_velocity_limit: The maximum angular velocity that can be provided

    -> (function, function)
    """

    #Check user input types
    assert isinstance(projection_distance, (int, float)), "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    assert isinstance(angular_velocity_limit, (int, float)), "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be an integer or float. Recieved type %r." % type(angular_velocity_limit).__name__
    
    #Check user input ranges/sizes
    assert projection_distance > 0, "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be positive. Recieved %r." % projection_distance
    assert projection_distance >= 0, "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be greater than or equal to zero. Recieved %r." % angular_velocity_limit

    def si_to_bi_dyn(dxi, poses):
        """Takes single-integrator velocities and transforms them to unicycle
        control inputs.

        dxi: 2xN numpy array of single-integrator control inputs
        poses: 4xN numpy array of unicycle poses

        -> 2xN numpy array of unicycle control inputs
        """

        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert poses.shape[0] == 4, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxi.shape[1] == poses.shape[1], "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size %r x %r and current pose array of size %r x %r." % (dxi.shape[0], dxi.shape[1], poses.shape[0], poses.shape[1])


        M,N = np.shape(dxi)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        v = np.zeros((1, N))
        v[0, :] = np.sqrt(dxi[0, :]**2 + dxi[1, :]**2)
        v[0, v[0,:]>max_speed] = max_speed
        v[0, v[0,:]<min_speed] = min_speed
        dxu[0, :] = (v[0, :] - poses[3, :])/dt
        dxu[1, :] = normalize_angle_array(np.arctan2(normalize_angle_array(normalize_angle_array(np.arctan2(dxi[1, :] , dxi[0, :]))-poses[2, :])* L, (poses[3,:]+0.0001)*dt))

        #Impose steering cap.
        dxu[1,dxu[1,:]>max_steer] = max_steer
        dxu[1,dxu[1,:]<-max_steer] = -max_steer 

        """# decreasing the desired speed when turning
        desired_speed = np.zeros((1,N))
        desired_speed[0, dxu[1,:]>math.radians(10)] = 3
        desired_speed[0, dxu[1,:]<-math.radians(10)] = 3
        desired_speed[0, dxu[1,:]<math.radians(10)] = 6
        desired_speed[0, dxu[1,:]>-math.radians(10)] = 6

        dxu[0, :] = 3 * (desired_speed-v[0,:])"""
        dxu[1, :] = dxu[1,:]
        return dxu

    def bi_to_si_states(poses):
        """Takes unicycle states and returns single-integrator states

        poses: 4xN numpy array of unicycle states

        -> 2xN numpy array of single-integrator states
        """

        _,N = np.shape(poses)

        si_states = np.zeros((2, N))
        si_states[0, :] = poses[0, :] + poses[3, :]*np.cos(poses[2, :])*dt
        si_states[1, :] = poses[1, :] + poses[3, :]*np.sin(poses[2, :])*dt

        return si_states

    return si_to_bi_dyn, bi_to_si_states

def create_bi_to_si_dynamics(projection_distance=0.05):
    """Creates two functions for mapping from unicycle dynamics to single 
    integrator dynamics and single integrator states to unicycle states. 
    
    This mapping is done by placing a virtual control "point" in front of 
    the unicycle.

    projection_distance: How far ahead to place the point

    -> function
    """

    #Check user input types
    assert isinstance(projection_distance, (int, float)), "In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    
    #Check user input ranges/sizes
    assert projection_distance > 0, "In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be positive. Recieved %r." % projection_distance
    

    def bi_to_si_dyn(dxu, poses):
        """A function for converting from unicycle to single-integrator dynamics.
        Utilizes a virtual point placed in front of the unicycle.

        dxu: 2xN numpy array of unicycle control inputs
        poses: 3xN numpy array of unicycle poses
        projection_distance: How far ahead of the unicycle model to place the point

        -> 2xN numpy array of single-integrator control inputs
        """

        #Check user input types
        assert isinstance(dxu, np.ndarray), "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the unicycle velocity inputs (dxu) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxu.shape[0] == 2, "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the unicycle velocity inputs (dxu) must be 2 ([v;w]). Recieved dimension %r." % dxu.shape[0]
        assert poses.shape[0] == 4, "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxu.shape[1] == poses.shape[1], "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the number of unicycle velocity inputs must be equal to the number of current robot poses. Recieved a unicycle velocity input array of size %r x %r and current pose array of size %r x %r." % (dxu.shape[0], dxu.shape[1], poses.shape[0], poses.shape[1])

        
        M,N = np.shape(dxu)
        v = np.zeros((1,N))
        yaw = np.zeros((1,N))
        dxu[1, :] = dxu[1, :]

        # v[0, :] = dxu[0, :]*dt
        v[0, :] = poses[3, :] + dxu[0, :]*dt
        yaw[0,:] = poses[2, :] + poses[3, :] * np.tan(dxu[1, :]) / L * dt

        cs = np.cos(yaw[0, :])
        ss = np.sin(yaw[0, :])

        dxi = np.zeros((2, N))
        dxi[0, :] = v[0, :]*cs
        dxi[1, :] = v[0, :]*ss

        return dxi

    return bi_to_si_dyn

def main(args=None):
    # Instantiate Robotarium object
    N = 2

    # The robots will never reach their goal points so set iteration number
    iterations = 3000

    # Define goal points outside of the arena
    goal_points = np.array(np.mat('5 5 5 5 5; 5 5 5 5 5; 0 0 0 0 0'))

    # Create barrier certificates to avoid collision
    uni_barrier_cert = create_unicycle_barrier_certificate_with_boundary()

    # define x initially --> state: [x, y, yaw, v]
    x = np.array([[0, 25], [0, 0], [0, np.pi], [0, 0]])
    goal1 = np.array([20, 10])
    goal2 = np.array([0, 10])
    cmd1 = ControlInputs()
    cmd2 = ControlInputs()

    trajectory, tx, ty = predict_trajectory(array_to_state(x[:,0]), goal1)
    trajectory2, tx2, ty2 = predict_trajectory(array_to_state(x[:,1]), goal2)

    # While the number of robots at the required poses is less
    # than N...
    for i in range(iterations):

        # Create single-integrator control inputs
        x1 = x[:,0]
        x2 = x[:, 1]
        x1 = array_to_state(x1)
        x2 = array_to_state(x2)
        
        dxu = np.zeros((2,N))
        dxu[0,0], dxu[1,0] = pure_pursuit_steer_control(goal1, x1)
        dxu[0,1], dxu[1,1] = pure_pursuit_steer_control(goal2, x2)

        # Create safe control inputs (i.e., no collisions)
        # print(dxu)
        dxu = uni_barrier_cert(dxu, x)
        # print(dxu)
        # print('\n')

        cmd1.throttle, cmd1.delta = dxu[0,0], dxu[1,0]
        cmd2.throttle, cmd2.delta = dxu[0,1], dxu[1,1]

        # Applying command and current state to the model
        x1 = linear_model_callback(x1, cmd1)
        x2 = linear_model_callback(x2, cmd2)

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(x1.x, x1.y, 'xk')
        plt.plot(x2.x, x2.y, 'xb')
        plot_arrow(x1.x, x1.y, x1.yaw)
        plot_arrow(x1.x, x1.y, x1.yaw + cmd1.delta)
        plot_arrow(x2.x, x2.y, x2.yaw)
        plot_arrow(x2.x, x2.y, x2.yaw + cmd2.delta)
        plot_path(trajectory)
        plot_path(trajectory2)
        plt.plot(goal1[0], goal1[1], '.k')
        plt.plot(goal2[0], goal2[1], '.b')

        # plot_map()
        plt.xlim(-50, 50)
        plt.ylim(-50, 50)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.000001)

        x1 = state_to_array(x1)
        x2 = state_to_array(x2)
        x = np.concatenate((x1, x2), axis=1)

if __name__=='__main__':
    main()
        
