#!/usr/bin/env python3

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ServoUdpBridge(Node):
    def __init__(self):
        super().__init__('servo_udp_bridge')
        self.declare_parameter('esp32_host', '192.168.0.42')
        self.declare_parameter('esp32_port', 8889)
        self.declare_parameter('pan_topic', '/servo_pan_cmd')
        self.declare_parameter('tilt_topic', '/servo_tilt_cmd')
        self.declare_parameter('initial_pan_us', 1500)
        self.declare_parameter('initial_tilt_us', 1650)

        self.esp32_host = self.get_parameter('esp32_host').value
        self.esp32_port = int(self.get_parameter('esp32_port').value)
        self.pan_us = self._clamp_us(self.get_parameter('initial_pan_us').value)
        self.tilt_us = self._clamp_us(self.get_parameter('initial_tilt_us').value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        pan_topic = self.get_parameter('pan_topic').value
        tilt_topic = self.get_parameter('tilt_topic').value
        self.pan_sub = self.create_subscription(
            Int32, pan_topic, self.pan_callback, 10)
        self.tilt_sub = self.create_subscription(
            Int32, tilt_topic, self.tilt_callback, 10)

        self.get_logger().info(
            f'ServoUdpBridge subscribed: pan={pan_topic}, tilt={tilt_topic}')
        self.get_logger().info(
            f'Servo UDP target: {self.esp32_host}:{self.esp32_port}')

    def pan_callback(self, msg):
        self.pan_us = self._clamp_us(msg.data)
        self.send_servo_command('pan')

    def tilt_callback(self, msg):
        self.tilt_us = self._clamp_us(msg.data)
        self.send_servo_command('tilt')

    def _clamp_us(self, value):
        return max(500, min(2500, int(value)))

    def send_servo_command(self, reason='update'):
        payload = f'{self.pan_us},{self.tilt_us}\n'.encode('ascii')
        try:
            self.sock.sendto(payload, (self.esp32_host, self.esp32_port))
            self.get_logger().info(
                f'Sent servo command ({reason}): pan={self.pan_us}, tilt={self.tilt_us}')
        except OSError as exc:
            self.get_logger().error(f'ESP32 UDP send failed: {exc}')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoUdpBridge()
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
