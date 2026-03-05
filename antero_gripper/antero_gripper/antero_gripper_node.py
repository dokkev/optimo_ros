#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, UInt8

from antero_gripper.canusb import CANUSB


class AnteroGripperNode(Node):
    def __init__(self):
        super().__init__('antero_gripper')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('rate', 25.0)

        port = self.get_parameter('port').value
        rate = float(self.get_parameter('rate').value)

        # Safety state
        self.status_safe = True
        self.pose_safe = True

        # Latest CAN command from bag playback
        self.latest_cmd = None

        # Publisher
        self.grip_status_pub = self.create_publisher(UInt8, '~/grip_status', 10)

        # Subscriptions
        self.create_subscription(
            Bool, '/optimo/safety_monitor/status',
            self._status_cb, 10)
        self.create_subscription(
            Bool, '/optimo/safety_monitor/human_pose_safe',
            self._pose_safe_cb, 10)
        self.create_subscription(
            Int32MultiArray, 'can_sensor_data',
            self._can_cmd_cb, 10)

        # CAN adapter
        self.can = None
        try:
            self.can = CANUSB(port=port)
            self.get_logger().info(f'CANUSB opened on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open CANUSB on {port}: {e}')

        # Timer
        self.timer = self.create_timer(max(1e-3, 1.0 / rate), self._loop)
        self.get_logger().info(
            f'ANTERO Gripper node started (rate={rate} Hz, port={port})')

    def _status_cb(self, msg):
        self.status_safe = msg.data

    def _pose_safe_cb(self, msg):
        self.pose_safe = msg.data

    def _can_cmd_cb(self, msg):
        if len(msg.data) >= 3:
            self.latest_cmd = msg.data

    def _loop(self):
        is_safe = self.status_safe and self.pose_safe

        if not is_safe:
            # Unsafe: send zero frame to deactivate gripper
            packet = 0
        elif self.latest_cmd is not None:
            # Safe with bag data: reconstruct CAN byte
            pin15 = self.latest_cmd[0]
            mag = self.latest_cmd[1]
            btn23 = self.latest_cmd[2]
            packet = ((pin15 & 0x01) << 7) | (mag & 0x7F)
            if not btn23:
                packet = 0
        else:
            # Safe but no bag data yet
            packet = 0

        # Send CAN frame
        if self.can is not None:
            try:
                self.can.send_frame([packet])
            except Exception as e:
                self.get_logger().error(f'CAN send error: {e}', throttle_duration_sec=2.0)

        # Publish grip status
        status_msg = UInt8()
        status_msg.data = packet
        self.grip_status_pub.publish(status_msg)

    def destroy_node(self):
        if self.can is not None:
            self.can.close()
            self.get_logger().info('CANUSB closed')
        super().destroy_node()


def main():
    rclpy.init()
    node = AnteroGripperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
