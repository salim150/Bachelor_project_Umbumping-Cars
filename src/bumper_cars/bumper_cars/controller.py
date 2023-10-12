#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
from custom_message.msg import ControlInputs, State, Path, Coordinate
from std_msgs.msg import Float32MultiArray
import message_filters
import random
import math
import numpy as np
from planner.cubic_spline_planner import *
from planner.frenet import *
from planner.predict_traj import *

debug = False
WB = 2.9  # [m] Wheel base of vehicle
Lf = 20  # [m] look-ahead distance
max_steer = np.radians(30.0)  # [rad] max steering angle


class Controller(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.previous_x = 0
        
        # creating random walk path for car2 to follow
        self.safety = 20 # safety border around the map boundaries
        self.width = 100.0 - self.safety
        self.heigth = 100.0 -self.safety
        print(type(self.width))

        self.path1 = []
        self.path1 = self.create_path(self.path1)
        self.target1 = (self.path1[0].x, self.path1[0].y)
        self.path2 = []
        self.path2 = self.create_path(self.path2)
        self.target2 = (self.path2[0].x, self.path2[0].y)
        self.trajectory, self.tx, self.ty = predict_trajectory(State(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0), self.target2)

        # Just to try if frenet path works
        # initial state
        self.c_speed = 10.0 / 3.6  # current speed [m/s]
        self.c_accel = 0.0  # current acceleration [m/ss]
        self.c_d = 2.0  # current lateral position [m]
        self.c_d_d = 0.0  # current lateral speed [m/s]
        self.c_d_dd = 0.0  # current lateral acceleration [m/s]
        self.s0 = 0.0  # current course position
        self.area = 50.0  # animation area length [m]
    
        self.control1_publisher_ = self.create_publisher(ControlInputs, "/robot1_control", 20)
        self.control2_publisher_ = self.create_publisher(ControlInputs, "/robot2_control", 20)
        self.path_pub = self.create_publisher(Path, "/robot2_path", 2)
        self.trajectory_pub = self.create_publisher(Path, "/robot2_trajectory", 2)
        
        state1_subscriber = message_filters.Subscriber(self, State, "/robot1_measurement")
        state2_subscriber = message_filters.Subscriber(self, State, "/robot2_measurement")

        ts = message_filters.ApproximateTimeSynchronizer([state1_subscriber, state2_subscriber], 2, 0.1, allow_headerless=True)
        ts.registerCallback(self.general_pose_callback)

        self.control1_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))
        self.control2_publisher_.publish(ControlInputs(delta=0.0, throttle=0.0))
        self.path_pub.publish(Path(path=self.path2))

        self.get_logger().info("Controller has been started")

    def general_pose_callback(self, state1: State, state2: State):
        
        self.pose1_callback(state1)
        self.pose2_callback(state2, state1)
        
        
    def pose1_callback(self, pose: State):
        # updating target waypoint and predicting new traj
        if self.dist(point1=(pose.x, pose.y), point2=self.target1) < Lf:
            self.path1 = self.update_path(self.path1)
            self.target1 = (self.path1[0].x, self.path1[0].y)
    
        cmd = ControlInputs()
        cmd.throttle, cmd.delta, self.path1, self.target1 = self.pure_pursuit_steer_control(self.target1, pose, self.path1)
        self.control1_publisher_.publish(cmd)
        
        if debug:
            self.get_logger().info("Control input robot1, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))

    def pose2_callback(self, pose: State, other_pose: State):

        ob = np.array([[other_pose.x, other_pose.y]])
        
        """tx, ty, tyaw, tc, csp = generate_target_course(self.tx, self.ty)
        path_opt = frenet_optimal_planning(csp, self.s0, self.c_speed, self.c_accel, self.c_d, self.c_d_d, self.c_d_dd, ob)
        #self.s0 = path_opt.s[1]
        self.c_d = path_opt.d[1]
        self.c_d_d = path_opt.d_d[1]
        self.c_d_dd = path_opt.d_dd[1]
        self.c_speed = path_opt.s_d[1]
        self.c_accel = path_opt.s_dd[1]"""

        if debug:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            self.plot_path(self.trajectory)
            plt.plot(path_opt.x[1:], path_opt.y[1:], "-or")
            plt.plot(pose.x, pose.y, 'xk')
            plt.plot(other_pose.x, other_pose.y, 'xb')
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.000001)
        

        # updating target waypoint and predicting new traj
        if self.dist(point1=(pose.x, pose.y), point2=self.target2) < Lf:
            self.path2 = self.update_path(self.path2)
            self.target2 = (self.path2[0].x, self.path2[0].y)
            self.trajectory, self.tx, self.ty  = predict_trajectory(pose, self.target2)


        cmd = ControlInputs()       
        cmd.throttle, cmd.delta, self.path2, self.target2 = self.pure_pursuit_steer_control(self.target2, pose, self.path2)
        self.control2_publisher_.publish(cmd)
        self.path_pub.publish(Path(path=self.path2))
        self.trajectory_pub.publish(Path(path=self.trajectory))
        
        if debug:
            self.get_logger().info("Control input robot2, delta:" + str(cmd.delta) + " , throttle: " + str(cmd.throttle))
    
    def update_path(self, path: Path):
        path.pop(0)
        path.append(Coordinate(x=float(random.randint(-self.width/2, self.width/2)), y=float(random.randint(-self.heigth/2, self.heigth/2))))
        return path

    def plot_path(self, path: Path):
        x = []
        y = []
        for coord in path:
            x.append(coord.x)
            y.append(coord.y)
        plt.scatter(x, y, marker='.', s=10)
        plt.scatter(x[0], y[0], marker='x', s=20)

    def create_path(self, path):
        while len(path)<5:
            path.append(Coordinate(x=float(random.randint(-self.width/2, self.width/2)), y=float(random.randint(-self.heigth/2, self.heigth/2))))
        return path
    
    def pure_pursuit_steer_control(self, target, pose: Pose, path: Path):

        alpha = self.normalize_angle(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)

        # this if/else condition should fix the buf of the waypoint behind the car
        if alpha > np.pi/2.0:
            delta = max_steer
        elif alpha < -np.pi/2.0: 
            delta = -max_steer
        else:
            # ref: https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html
            delta = self.normalize_angle(math.atan2(2.0 * WB *  math.sin(alpha), Lf))
        
        # decreasing the desired speed when turning
        if delta > math.radians(10) or delta < -math.radians(10):
            desired_speed = 3
        else:
            desired_speed = 6

        delta = 3 * math.degrees(delta)
        throttle = 3 * (desired_speed-pose.v)

        return throttle, delta, path, target

    @staticmethod
    def dist(point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)

        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return distance

    def normalize_angle(self, angle):
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
            print(dxi)
            dxi = si_barrier_cert(dxi, x_si)
            print(dxi)
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
            uv = np.zeros((1, N))
            uw = np.zeros((1, N))

            uv[0, :] = (cs*dxi[0, :] + ss*dxi[1, :])
            uw[0, :] = (1/projection_distance)*(-ss*dxi[0, :] + cs*dxi[1, :])

            #Impose angular velocity cap.
            uw[0,uw[0,:]>angular_velocity_limit] = angular_velocity_limit
            uw[0,uw[0,:]<-angular_velocity_limit] = -angular_velocity_limit 

            uv[0, uv[0,:]>max_speed] = max_speed
            uv[0, uv[0,:]<min_speed] = min_speed

            dxu[0, :] = 3 * (uv[0, :] - poses[3, :])
            dxu[1, :] = np.arctan2(uw[0,:]* L, poses[3, :]+0.00001)
            #Impose steering cap.
            dxu[1,dxu[1,:]>max_steer] = max_steer
            dxu[1,dxu[1,:]<-max_steer] = -max_steer 

            """dxu = np.zeros((2, N))
            v = np.zeros((1, N))
            v[0, :] = np.sqrt(dxi[0, :]**2 + dxi[1, :]**2)
            v[0, v[0,:]>max_speed] = max_speed
            v[0, v[0,:]<min_speed] = min_speed
            dxu[0, :] = 3 * (v[0, :] - poses[3, :])
            dxu[1, :] = np.arctan2((np.arctan2(dxi[1, :]+0.0001 , dxi[0, :]+0.0001)-poses[2, :])* L, poses[3, :]+0.0001)

            #Impose angular velocity cap.
            dxu[1,dxu[1,:]>max_steer] = max_steer
            dxu[1,dxu[1,:]<-max_steer] = -max_steer 

            # decreasing the desired speed when turning
            desired_speed = np.zeros((1,N))
            desired_speed[0, dxu[1,:]>math.radians(10)] = 3
            desired_speed[0, dxu[1,:]<-math.radians(10)] = 3
            desired_speed[0, dxu[1,:]<math.radians(10)] = 4
            desired_speed[0, dxu[1,:]>-math.radians(10)] = 4

            input_des = np.zeros((1,N))
            input_des[0, :] = 3 * (desired_speed - v[0, :])
            dxu[0, :] = 3 * (desired_speed - poses[3, :])"""
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

            
            """M,N = np.shape(dxu)
            v = np.zeros((1,N))
            # v[0, :] = dxu[0, :]*dt
            v[0, :] = dxu[0, :]*dt + poses[3, :]

            cs = np.cos(v[0, :] * np.tan(dxu[1, :]) / L)
            ss = np.sin(v[0, :] * np.tan(dxu[1, :]) / L)

            dxi = np.zeros((2, N))
            dxi[0, :] = v[0, :]*cs
            dxi[1, :] = v[0, :]*ss"""

            M,N = np.shape(dxu)

            cs = np.cos(poses[2, :])
            ss = np.sin(poses[2, :])

            us = np.zeros((1,N))
            uw = np.zeros((1,N))

            us[0,:] = dxu[0, :]*dt
            uw[0,:] = us[0,:] * np.tan(dxu[1,:]) / L

            dxi = np.zeros((2, N))
            dxi[0, :] = (cs*us[0,:] - projection_distance*ss*uw[0, :])
            dxi[1, :] = (ss*us[0, :] + projection_distance*cs*uw[0, :])

            return dxi

        return bi_to_si_dyn
      

def main(args=None):
    rclpy.init(args=args)

    node = Controller()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()