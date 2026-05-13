import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('smart_car_py_pkg'),
        'config',
        'webcam_params.yaml',
    )
    esp32_host = LaunchConfiguration('esp32_host')
    esp32_port = LaunchConfiguration('esp32_port')

    webcam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('image_raw', '/webcam/image_raw'),
            ('camera_info', '/webcam/camera_info'),
        ],
    )

    webcam2_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam2',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('image_raw', '/webcam2/image_raw'),
            ('camera_info', '/webcam2/camera_info'),
        ],
    )

    webcam_compressed_node = Node(
        package='image_transport',
        executable='republish',
        name='webcam_compressed_republisher',
        output='screen',
        arguments=[
            'raw',
            'compressed',
        ],
        remappings=[
            ('in', '/webcam/image_raw'),
            ('out/compressed', '/webcam/image_raw/compressed'),
        ],
    )

    webcam2_compressed_node = Node(
        package='image_transport',
        executable='republish',
        name='webcam2_compressed_republisher',
        output='screen',
        arguments=[
            'raw',
            'compressed',
        ],
        remappings=[
            ('in', '/webcam2/image_raw'),
            ('out/compressed', '/webcam2/image_raw/compressed'),
        ],
    )

    servo_udp_bridge_node = Node(
        package='smart_car_py_pkg',
        executable='servo_udp_bridge',
        name='servo_udp_bridge',
        output='screen',
        parameters=[{
            'esp32_host': esp32_host,
            'esp32_port': esp32_port,
            'pan_topic': '/servo_pan_cmd',
            'tilt_topic': '/servo_tilt_cmd',
            'initial_tilt_us': 1650,
        }],
    )

    turtlebot_sound_bridge_node = Node(
        package='smart_car_py_pkg',
        executable='turtlebot_sound_bridge',
        name='turtlebot_sound_bridge',
        output='screen',
        parameters=[{
            'command_topic': '/learning_complete_sound_cmd',
            'sound_topic': '/sound',
            'sound_value': 1,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'esp32_host',
            default_value='192.168.0.42',
            description='ESP32 servo controller IP address',
        ),
        DeclareLaunchArgument(
            'esp32_port',
            default_value='8889',
            description='ESP32 servo controller UDP port',
        ),
        webcam_node,
        webcam2_node,
        webcam_compressed_node,
        webcam2_compressed_node,
        servo_udp_bridge_node,
        turtlebot_sound_bridge_node,
    ])
