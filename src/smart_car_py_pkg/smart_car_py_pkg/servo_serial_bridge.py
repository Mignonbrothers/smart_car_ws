#!/usr/bin/env python3

import os
import termios

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ServoSerialBridge(Node):
    def __init__(self):
        super().__init__('servo_serial_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('topic', '/servo_cmd')

        self.serial_fd = None
        self._open_serial()

        topic = self.get_parameter('topic').value
        self.sub = self.create_subscription(Int32, topic, self.servo_callback, 10)
        self.get_logger().info(f'ServoSerialBridge subscribed: {topic}')

    def _open_serial(self):
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baud').value)
        baud_const = getattr(termios, f'B{baud}', None)
        if baud_const is None:
            self.get_logger().error(f'Unsupported serial baud rate: {baud}')
            return

        try:
            self.serial_fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(self.serial_fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = baud_const
            attrs[5] = baud_const
            termios.tcsetattr(self.serial_fd, termios.TCSANOW, attrs)
            termios.tcflush(self.serial_fd, termios.TCIOFLUSH)
            self.get_logger().info(f'OpenCR serial connected: {port} @ {baud}')
        except Exception as exc:
            self.serial_fd = None
            self.get_logger().error(f'OpenCR serial open failed: {port} ({exc})')

    def servo_callback(self, msg):
        if self.serial_fd is None:
            return

        value = int(msg.data)
        value = max(500, min(2500, value))
        try:
            os.write(self.serial_fd, f'{value}\n'.encode('ascii'))
        except Exception as exc:
            self.get_logger().error(f'OpenCR serial write failed: {exc}')
            try:
                os.close(self.serial_fd)
            except Exception:
                pass
            self.serial_fd = None

    def destroy_node(self):
        if self.serial_fd is not None:
            try:
                os.close(self.serial_fd)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialBridge()
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
