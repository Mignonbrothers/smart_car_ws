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
    serial_port = LaunchConfiguration('serial_port')

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

    servo_serial_bridge_node = Node(
        package='smart_car_py_pkg',
        executable='servo_serial_bridge',
        name='servo_serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baud': 115200,
            'topic': '/servo_cmd',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='OpenCR serial device path',
        ),
        webcam_node,
        webcam2_node,
        webcam_compressed_node,
        webcam2_compressed_node,
        servo_serial_bridge_node,
    ])
