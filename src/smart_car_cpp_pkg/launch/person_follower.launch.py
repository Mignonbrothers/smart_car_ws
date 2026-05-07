import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('smart_car_cpp_pkg'),
        'config',
        'person_follower.yaml',
    )

    person_follower_node = Node(
        package='smart_car_cpp_pkg',
        executable='person_follower',
        name='person_follower',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        person_follower_node,
    ])
