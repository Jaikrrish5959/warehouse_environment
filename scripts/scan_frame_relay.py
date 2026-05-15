#!/usr/bin/env python3
"""
Sensor Relay Node
- Subscribes to /scan, rewrites frame_id to 'laser_frame', republishes to /scan_fixed
- This fixes Gazebo Sim's namespaced frame 'articubot/base_link/laser' so RViz/Nav2 can use it
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameRelay(Node):
    def __init__(self):
        super().__init__('scan_frame_relay')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.sub = self.create_subscription(
            LaserScan, '/scan_raw', self.cb, sensor_qos)
        self.get_logger().info('Scan frame relay: /scan_raw -> /scan (frame_id=laser_frame)')

    def cb(self, msg: LaserScan):
        msg.header.frame_id = 'laser_frame'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
