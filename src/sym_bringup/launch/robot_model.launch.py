from launch import LaunchDescription
from launch_ros.actions import Node
from custom_message.msg import ControlInputs, State, FullState

def generate_launch_description():
    ld = LaunchDescription()

    robot_node = Node(
        package="bumper_cars",
        executable="model",
        name="robot_models",
        remappings=[
            ("/robot_control", "/multi_control"),
            ("/robot_fullstate", "/multi_fullstate")
        ],
        parameters=[
            {'model_type': ['linear', 'linear', 'linear', 'linear']},
            {'x0': [30.0, 0.0, -30.0, 0.0]},
            {'y0': [30.0, 0.0, -30.0, -20.0]},
            {'yaw': [0.0, 0.0, 0.0, 0.0]},
            {'v': [0.0, 0.0, 0.0, 0.0]},
            {'omega': [0.0, 0.0, 0.0, 0.0]}
            ]
        )
    
    ld.add_action(robot_node)  

    return ld