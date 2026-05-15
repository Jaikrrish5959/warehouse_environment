#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Warehouse Robot!
---------------------------
Moving around:
        W
   A    S    D

W/S : Increase/Decrease Linear Velocity (Forward/Backward)
A/D : Increase/Decrease Angular Velocity (Turn Left/Right)

Q/E : Speed up/down by 10%
Spacebar : Force Stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'e': (0.9, 0.9),
}

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.th = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            print(msg)
            while True:
                key = self.get_key()
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    print(f"Currently: speed {self.speed:.2f} | turn {self.turn:.2f}")
                elif key == ' ':
                    self.x = 0.0
                    self.th = 0.0
                elif key == '\x03': # CTRL-C
                    break
                else:
                    self.x = 0.0
                    self.th = 0.0

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.angular.z = self.th * self.turn
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleop()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
