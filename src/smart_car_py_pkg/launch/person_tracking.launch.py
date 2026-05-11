import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    person_follower_config = os.path.join(
        get_package_share_directory('smart_car_cpp_pkg'),
        'config',
        'person_follower.yaml',
    )

    pan_tilt_node = Node(
        package='smart_car_py_pkg',
        executable='pan_tilt_ros2',
        name='smart_cart_tracker_pc',
        output='screen',
        parameters=[{
            'start_enabled': True,
        }],
    )

    person_follower_node = Node(
        package='smart_car_cpp_pkg',
        executable='person_follower',
        name='person_follower',
        output='screen',
        parameters=[
            person_follower_config,
            {
                'start_enabled': True,
                'command_topic': '/gui_command',
            },
        ],
    )

    return LaunchDescription([
        pan_tilt_node,
        person_follower_node,
    ])
