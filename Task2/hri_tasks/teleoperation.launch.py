from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    # Parameter file
    param_file = os.path.join(
        os.path.dirname(__file__),
        'teleop_params.yaml'
    )

    return LaunchDescription([

        # FSR sensor simulator
        Node(
            package='ros2_ur5_sim',     # your repo name if using setup.py
            executable='fsr_sensor.py', # only works if installed OR run with ros2 run
            name='fsr_sensor',
            output='screen',
            parameters=[param_file]
        ),

        # force mapping node
        Node(
            package='ros2_ur5_sim',
            executable='mapping_node.py',
            name='force_mapping_node',
            output='screen',
            parameters=[param_file]
        ),

        # teleoperation node
        Node(
            package='ros2_ur5_sim',
            executable='teleoperation.py',
            name='teleoperation_node',
            output='screen'
        ),
    ])
