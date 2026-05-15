#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

import sys, select, termios, tty
import threading

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.safe_distance = 0.5  # safe distance in meters
        self.min_front_distance = float('inf')
        self.min_rear_distance = float('inf')
        self.get_logger().info('Keyboard control node started. Use W/A/S/D to move. Press CTRL-C to exit.')

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        front_ranges = []
        rear_ranges = []
        for i, range_val in enumerate(msg.ranges):
            angle = angle_min + i * angle_inc
            if -math.pi/4 <= angle <= math.pi/4:
                if not math.isinf(range_val) and not math.isnan(range_val):
                    front_ranges.append(range_val)
            elif angle >= 3*math.pi/4 or angle <= -3*math.pi/4:
                if not math.isinf(range_val) and not math.isnan(range_val):
                    rear_ranges.append(range_val)
                    
        if front_ranges:
            self.min_front_distance = min(front_ranges)
        else:
            self.min_front_distance = float('inf')
            
        if rear_ranges:
            self.min_rear_distance = min(rear_ranges)
        else:
            self.min_rear_distance = float('inf')

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        settings = termios.tcgetattr(sys.stdin)
    except termios.error:
        print("Failed to get terminal settings. Are you running this in a real terminal?")
        settings = None

    try:
        while rclpy.ok():
            if settings:
                key = getKey(settings)
            else:
                key = ''
                
            twist = Twist()
            
            if key == 'w':
                if node.min_front_distance < node.safe_distance:
                    print(f"\r\nobject {node.min_front_distance:.2f}m is in the front cannot go")
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = 1.0
            elif key == 's':
                if node.min_rear_distance < node.safe_distance:
                    print(f"\r\nobject {node.min_rear_distance:.2f}m is in the back cannot go")
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = -1.0
            elif key == 'a':
                twist.angular.z = 2.0
            elif key == 'd':
                twist.angular.z = -2.0
            elif key == '\x03': # CTRL-C
                break
                
            # If no key, it publishes 0,0 which stops the robot
            node.cmd_pub.publish(twist)
            
    except Exception as e:
        print(e)
    finally:
        if settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
