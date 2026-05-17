#!/usr/bin/env python3
"""
Proximity Monitor Node
Subscribes to /scan (LaserScan) and prints colour-coded terminal warnings
when the robot is close to obstacles on any side.

Sectors (based on 360° scan, 0° = front):
  FRONT  : -30° to +30°
  LEFT   : +30° to +150°
  BACK   : +150° to -150° (±180°)
  RIGHT  : -150° to -30°
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


# ANSI colour codes
RED     = '\033[91m'
YELLOW  = '\033[93m'
GREEN   = '\033[92m'
CYAN    = '\033[96m'
BOLD    = '\033[1m'
RESET   = '\033[0m'


class ProximityMonitor(Node):
    # Distance thresholds (metres)
    DANGER_DIST  = 0.5
    WARNING_DIST = 1.0
    CAUTION_DIST = 2.0

    def __init__(self):
        super().__init__('proximity_monitor')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.get_logger().info(
            f'{GREEN}Proximity Monitor started — listening on /scan{RESET}')
        # Throttle printing to ~2 Hz
        self.last_print_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds < 500_000_000:  # 0.5 s
            return
        self.last_print_time = now

        if len(msg.ranges) == 0:
            return

        n = len(msg.ranges)
        angle_inc = msg.angle_increment
        angle_min = msg.angle_min

        # Build sector bins
        front, back, left, right = [], [], [], []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                continue
            angle = angle_min + i * angle_inc  # radians, 0 = front

            # Normalise to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))

            deg = math.degrees(angle)
            if -30 <= deg <= 30:
                front.append(r)
            elif 30 < deg <= 150:
                left.append(r)
            elif -150 <= deg < -30:
                right.append(r)
            else:
                back.append(r)

        # Compute minimum distance per sector
        sectors = {
            'FRONT': min(front) if front else float('inf'),
            'BACK':  min(back)  if back  else float('inf'),
            'LEFT':  min(left)  if left  else float('inf'),
            'RIGHT': min(right) if right else float('inf'),
        }

        # Header
        lines = [f'\n{BOLD}{"═"*50}']
        lines.append(f'  🤖  PROXIMITY MONITOR')
        lines.append(f'{"═"*50}{RESET}')

        any_warning = False
        for name, dist in sectors.items():
            tag, colour = self._classify(dist)
            icon = self._icon(name)
            if dist == float('inf'):
                lines.append(f'  {icon} {name:6s}  {GREEN}CLEAR{RESET}')
            else:
                lines.append(
                    f'  {icon} {name:6s}  {colour}{tag:8s}{RESET}  '
                    f'{dist:.2f} m')
                if tag in ('DANGER', 'WARNING'):
                    any_warning = True

        lines.append(f'{BOLD}{"═"*50}{RESET}')

        if any_warning:
            print('\n'.join(lines))

    @staticmethod
    def _classify(dist):
        if dist <= ProximityMonitor.DANGER_DIST:
            return 'DANGER', RED + BOLD
        elif dist <= ProximityMonitor.WARNING_DIST:
            return 'WARNING', YELLOW + BOLD
        elif dist <= ProximityMonitor.CAUTION_DIST:
            return 'CAUTION', YELLOW
        else:
            return 'OK', GREEN

    @staticmethod
    def _icon(sector):
        return {'FRONT': '⬆️ ', 'BACK': '⬇️ ', 'LEFT': '⬅️ ', 'RIGHT': '➡️ '}[sector]


def main(args=None):
    rclpy.init(args=args)
    node = ProximityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
