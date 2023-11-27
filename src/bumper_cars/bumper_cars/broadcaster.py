import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from custom_message.msg import MultiState, FullState, State, MultiplePaths

# For the parameter file
import pathlib
import json
from geometry_msgs.msg import Pose
import numpy as np

path = pathlib.Path('/home/giacomo/thesis_ws/src/bumper_cars/params.json')
# Opening JSON file
with open(path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)

plot_traj = json_object["plot_traj"]
robot_num = json_object["robot_num"]
timer_freq = json_object["timer_freq"]

class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster_node')
        self.subscription = self.create_subscription(
            MultiState,
            '/multi_fullstate',
            self.state_callback,
            10
        )
        self.transform_broadcaster = self.create_publisher(
            TransformStamped,
            '/tf',
            10
        )
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
        
    def convert_to_pose(self, state: FullState):
        """
        Convert a robot state to a pose.

        Input
            :param state: The robot state.

        Output
            :return pose: The converted pose.
        """
        pose = Pose()
        pose.position.x = state.x
        pose.position.y = state.y
        pose.position.z = 0.0

        q = self.get_quaternion_from_euler(0, 0, state.yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def state_callback(self, msg: MultiState):
        for i in range(robot_num):
            pose = self.convert_to_pose(msg.multiple_state[i])
            # Convert state message to transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = f'robot_frame_{i}'
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = pose.orientation.x
            transform.transform.rotation.y = pose.orientation.y
            transform.transform.rotation.z = pose.orientation.z
            transform.transform.rotation.w = pose.orientation.w

            # Publish the transform
            self.transform_broadcaster.publish(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcasterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
