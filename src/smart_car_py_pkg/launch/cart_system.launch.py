from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    cart_api_url = LaunchConfiguration('cart_api_url')
    confidence = LaunchConfiguration('confidence')
    cooldown = LaunchConfiguration('cooldown')

    cart_gui_node = Node(
        package='smart_car_py_pkg',
        executable='cart_gui',
        name='cart_gui',
        output='screen',
    )

    ros2_cart_bridge_node = Node(
        package='smart_car_py_pkg',
        executable='ros2_cart_bridge',
        name='ros2_cart_bridge',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'cart_api_url': cart_api_url,
            'confidence': confidence,
            'cooldown': cooldown,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/webcam/image_raw/compressed',
            description='Compressed camera image topic for product recognition',
        ),
        DeclareLaunchArgument(
            'cart_api_url',
            default_value='http://127.0.0.1:5000/api/add_item',
            description='Smart cart Flask API endpoint',
        ),
        DeclareLaunchArgument(
            'confidence',
            default_value='0.6',
            description='YOLO detection confidence threshold',
        ),
        DeclareLaunchArgument(
            'cooldown',
            default_value='3.0',
            description='Seconds before sending the same class again',
        ),
        cart_gui_node,
        ros2_cart_bridge_node,
    ])
