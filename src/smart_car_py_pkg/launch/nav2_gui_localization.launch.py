import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_nav2_bringup(context):
    nav2_share = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_share, 'launch', 'bringup_launch.py')

    launch_arguments = {
        'map': LaunchConfiguration('map').perform(context),
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
        'autostart': LaunchConfiguration('autostart').perform(context),
        'params_file': LaunchConfiguration('params_file').perform(context),
        'slam': LaunchConfiguration('slam').perform(context),
        'use_composition': LaunchConfiguration('use_composition').perform(context),
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    smart_car_share = get_package_share_directory('smart_car_py_pkg')
    smart_car_cpp_share = get_package_share_directory('smart_car_cpp_pkg')
    default_map = os.path.join(
        smart_car_share,
        'gui',
        'maps',
        'map.yaml',
    )
    default_params_file = os.path.join(
        smart_car_share,
        'config',
        'nav2_params.yaml',
    )

    robot_gui_node = Node(
        package='smart_car_py_pkg',
        executable='robot_gui',
        name='robot_gui',
        output='screen',
    )

    go_to_pose_node = Node(
        package='smart_car_cpp_pkg',
        executable='go_to_pose',
        name='go_to_pose_cpp',
        output='screen',
        parameters=[{
            'command_topic': '/gui_command',
            'frame_id': 'map',
            'action_name': 'navigate_to_pose',
        }],
    )

    pan_tilt_node = Node(
        package='smart_car_py_pkg',
        executable='pan_tilt_ros2',
        name='smart_cart_tracker_pc',
        output='screen',
    )

    person_follower_node = Node(
        package='smart_car_cpp_pkg',
        executable='person_follower',
        name='person_follower',
        output='screen',
        parameters=[
            os.path.join(smart_car_cpp_share, 'config', 'person_follower.yaml'),
            {'command_topic': '/gui_command'},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Map yaml file for Nav2 AMCL localization.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 params yaml.',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Use SLAM instead of loading a static map.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed Nav2 bringup if true.',
        ),
        OpaqueFunction(function=include_nav2_bringup),
        robot_gui_node,
        go_to_pose_node,
        pan_tilt_node,
        person_follower_node,
    ])
