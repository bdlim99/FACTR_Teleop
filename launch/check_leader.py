from ament_index_python.packages import get_package_share_directory
import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_description = xacro.process_file(
        os.path.join(
            get_package_share_directory('franka_description'), 'robots', 'panda_arm.urdf.xacro'
        ),
        mappings={'hand': 'true'}
    ).toprettyxml(indent='  ')

    rviz_file = os.path.join(
        get_package_share_directory('franka_description'), 'rviz', 'visualize_franka.rviz'
    )

    return LaunchDescription([
        Node(
            package='factr_teleop',
            executable='check_leader',
            output='screen',
            emulate_tty=True,
            parameters=[{'config_file': 'check_leader.yaml'}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['--display-config', rviz_file]
        )
    ])
