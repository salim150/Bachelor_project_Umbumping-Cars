#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from custom_message.msg import ControlInputs

class CarModel(Node):

    def __init__(self):
        super().__init__("car_model")

        self.measurement_publisher_ = self.create_publisher(Pose, "/robot_state", 10)
        self.state_subscriber_ = self.create_subscription(Pose,
                                                         "/turtle1/pose", qos_profile=10) 
        self.control_subscriber_ = self.create_subscription(ControlInputs,
                                                         "/turtle1/pose", 
                                                         self.model_callback, 10) 
        # TODO: change the topics name and create new ones. the subscriber of this class should be subscribed to the /control_input topic and publishing the 
        # /current_pose topic
        
        self.get_logger().info("Car model started succesfully")
        
    def model_callback(self, pose: Pose, cmd: ControlInputs):
        
        
        

def main(args=None):
    rclpy.init(args=args)

    node = CarModel()
    rclpy.spin(node)

    rclpy.shutdown()