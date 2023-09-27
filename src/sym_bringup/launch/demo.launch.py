from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_node = Node(
        package="bumper_cars",
        executable="controller",
    )

    sensor_node = Node(
        package="bumper_cars",
        executable="sensor"
    )

    plotter_node = Node(
        package="bumper_cars",
        executable="plotter"
    )
    
    ld.add_action(plotter_node)
    
    ld.add_action(sensor_node)
    ld.add_action(controller_node)
    #ld.add_action(robot_node)
    

    return ld