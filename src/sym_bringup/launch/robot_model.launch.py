from launch import LaunchDescription
from launch_ros.actions import Node
from custom_message.msg import ControlInputs, State, FullState

def generate_launch_description():
    ld = LaunchDescription()

    robot1_node = Node(
        package="bumper_cars",
        executable="model",
        name="robot1_model",
        remappings=[
            ("/robot_control", "/robot1_control"),
            ("/robot_fullstate", "/robot1_fullstate")
        ],
        parameters=[
            {'model_type': 'linear'},
            {'x0': 30.0},
            {'y0': 30.0},
            {'yaw': 0.0},
            {'v': 0.0},
            {'omega': 0.0}
            ]
        )

    robot2_node = Node(
        package="bumper_cars",
        executable="model",
        name="robot2_model",
        remappings=[
            ("/robot_control", "/robot2_control"),
            ("/robot_fullstate", "/robot2_fullstate")
        ],
        parameters=[
            {'model_type': 'linear'},
            {'x0': 0.0},
            {'y0': 0.0},
            {'yaw': 0.0},
            {'v': 0.0},
            {'omega': 0.0}
        ]
    )

    robot3_node = Node(
        package="bumper_cars",
        executable="model",
        name="robot3_model",
        remappings=[
            ("/robot_control", "/robot3_control"),
            ("/robot_fullstate", "/robot3_fullstate")
        ],
        parameters=[
            {'model_type': 'linear'},
            {'x0': -30.0},
            {'y0': -30.0},
            {'yaw': 0.0},
            {'v': 0.0},
            {'omega': 0.0}
        ]
    )

    robot4_node = Node(
        package="bumper_cars",
        executable="model",
        name="robot4_model",
        remappings=[
            ("/robot_control", "/robot4_control"),
            ("/robot_fullstate", "/robot4_fullstate")
        ],
        parameters=[
            {'model_type': 'linear'},
            {'x0': 0.0},
            {'y0': -20.0},
            {'yaw': 0.0},
            {'v': 0.0},
            {'omega': 0.0}
        ]
    )
    
    ld.add_action(robot1_node)
    ld.add_action(robot2_node)  
    ld.add_action(robot3_node) 
    ld.add_action(robot4_node)    

    return ld