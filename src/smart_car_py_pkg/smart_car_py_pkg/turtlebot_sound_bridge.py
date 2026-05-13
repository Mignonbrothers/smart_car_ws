import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from turtlebot3_msgs.msg import Sound


class TurtleBotSoundBridge(Node):
    def __init__(self):
        super().__init__('turtlebot_sound_bridge')
        self.declare_parameter('command_topic', '/learning_complete_sound_cmd')
        self.declare_parameter('sound_topic', '/sound')
        self.declare_parameter('sound_value', int(Sound.ON))

        self.sound_value = int(self.get_parameter('sound_value').value)
        self.stop_timer = None

        command_topic = self.get_parameter('command_topic').value
        sound_topic = self.get_parameter('sound_topic').value
        self.sound_pub = self.create_publisher(Sound, sound_topic, 10)
        self.command_sub = self.create_subscription(
            Int32,
            command_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info(
            f'TurtleBot sound bridge: {command_topic} -> {sound_topic}')

    def command_callback(self, msg):
        duration_ms = max(0, min(10000, int(msg.data)))
        if duration_ms <= 0:
            return

        self.publish_sound(self.sound_value)
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(duration_ms / 1000.0, self.stop_sound)
        self.get_logger().info(f'Sound ON for {duration_ms}ms')

    def publish_sound(self, value):
        msg = Sound()
        msg.value = int(value)
        self.sound_pub.publish(msg)

    def stop_sound(self):
        self.publish_sound(Sound.OFF)
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
        self.get_logger().info('Sound OFF')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotSoundBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
