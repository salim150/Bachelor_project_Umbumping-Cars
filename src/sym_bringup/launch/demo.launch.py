from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_node = Node(
        package="bumper_cars",
        executable="controller",
        parameters=[
            {'model_type': ['linear', 'linear', 'linear', 'linear']},
            {'x0': [30.0, 0.0, -30.0, 0.0]},
            {'y0': [30.0, 0.0, -30.0, -20.0]},
            {'yaw': [0.0, 0.0, -3.14, 0.0]},
            {'v': [0.0, 0.0, 0.0, 0.0]},
            {'omega': [0.0, 0.0, 0.0, 0.0]}
            ]
    )

    sensor_node = Node(
        package="bumper_cars",
        executable="sensor",
        parameters=[
            {'model_type': ['linear', 'linear', 'linear', 'linear']},
            {'x0': [30.0, 0.0, -30.0, 0.0]},
            {'y0': [30.0, 0.0, -30.0, -20.0]},
            {'yaw': [0.0, 0.0, -3.14, 0.0]},
            {'v': [0.0, 0.0, 0.0, 0.0]},
            {'omega': [0.0, 0.0, 0.0, 0.0]}
            ]
    )

    plotter_node = Node(
        package="bumper_cars",
        executable="plotter",
        parameters=[
            {'model_type': ['linear', 'linear', 'linear', 'linear']},
            {'x0': [30.0, 0.0, -30.0, 0.0]},
            {'y0': [30.0, 0.0, -30.0, -20.0]},
            {'yaw': [0.0, 0.0, -3.14, 0.0]},
            {'v': [0.0, 0.0, 0.0, 0.0]},
            {'omega': [0.0, 0.0, 0.0, 0.0]}
            ]
    )
    
    ld.add_action(plotter_node)
    ld.add_action(sensor_node)
    ld.add_action(controller_node)
    #ld.add_action(robot_node)
    

    return ld