#!/usr/bin/env python3
"""
Twist Stamper Node
- Subscribes to geometry_msgs/Twist on /cmd_vel_unstamped
- Publishes geometry_msgs/TwistStamped on /diff_cont/cmd_vel
- Adds current time and frame_id to the Twist message.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')

        self.pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.sub = self.create_subscription(
            Twist, '/cmd_vel_unstamped', self.cb, 10)
        self.get_logger().info('Twist stamper: /cmd_vel_unstamped -> /diff_cont/cmd_vel')

    def cb(self, msg: Twist):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        self.pub.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
