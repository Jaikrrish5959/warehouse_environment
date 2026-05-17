#!/usr/bin/env python3
"""
Collision Guard Node with Semantic Keepout Enforcement
======================================================
1. Subscribes to /scan (Lidar safety) and monitors direct physical obstacles.
2. Parses warehouse_ontology.owl to extract restricted zones (yellow-black markings).
3. Uses a tf2_ros listener to track the robot's high-precision coordinate in the map frame.
4. Triggers an emergency stop if the robot enters any OWL restricted zone.
5. Smart Escape: Allows manual teleop commands that steer AWAY from the restricted zone center.
"""

import os
import math
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory

# ANSI color codes
RED    = '\033[91m'
YELLOW = '\033[93m'
GREEN  = '\033[92m'
BOLD   = '\033[1m'
RESET  = '\033[0m'


class CollisionGuard(Node):
    STOP_DISTANCE = 0.45   # metres — lidar e-stop threshold
    WARN_DISTANCE = 0.70   # metres — lidar warning threshold
    ROBOT_RADIUS = 0.22    # metres — padding for restricted zones

    def __init__(self):
        super().__init__('collision_guard')

        # TF listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load keepout zones directly from OWL ontology
        self.restricted_zones = self.load_restricted_zones()

        # QoS for Lidar
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # Subscriptions & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_unstamped', self.cmd_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        # State Variables
        self.last_print = self.get_clock().now()
        self.last_cmd = Twist()
        self.stopped_by_lidar = False
        self.stopped_by_zone = False

        self.get_logger().info(f'{GREEN}Semantic Collision Guard initialized with {len(self.restricted_zones)} OWL zones.{RESET}')

    def load_restricted_zones(self):
        """Parses the OWL file and extracts restricted zone definitions."""
        zones = []
        try:
            # Locate ontology file
            try:
                share_dir = get_package_share_directory('warehouse_env')
                owl_path = os.path.join(share_dir, 'ontology', 'warehouse_ontology.owl')
            except Exception:
                owl_path = '/home/jaikrrishs/Desktop/robot_navigation/warehouse_robot/ontology/warehouse_ontology.owl'

            if not os.path.exists(owl_path):
                self.get_logger().warn(f"Ontology file not found at {owl_path}, using hardcoded layout.")
                raise FileNotFoundError()

            tree = ET.parse(owl_path)
            root = tree.getroot()

            # RDF/XML namespaces
            ns = {
                'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
                'owl': 'http://www.w3.org/2002/07/owl#',
                'rdfs': 'http://www.w3.org/2000/01/rdf-schema#',
                'wh': 'http://warehouse-robot.local/ontology#'
            }

            for ind in root.findall('.//owl:NamedIndividual', ns):
                type_elem = ind.find('./rdf:type', ns)
                if type_elem is not None and 'RestrictedZone' in type_elem.attrib.get('{http://www.w3.org/1999/02/22-rdf-syntax-ns#}resource', ''):
                    # Extract values
                    label_elem = ind.find('./rdfs:label', ns)
                    label = label_elem.text if label_elem is not None else ind.attrib.get('{http://www.w3.org/1999/02/22-rdf-syntax-ns#}about', '').split('#')[-1]

                    min_x = float(ind.find('.//wh:boundMinX', ns).text)
                    max_x = float(ind.find('.//wh:boundMaxX', ns).text)
                    min_y = float(ind.find('.//wh:boundMinY', ns).text)
                    max_y = float(ind.find('.//wh:boundMaxY', ns).text)

                    zones.append({
                        'label': label,
                        'min_x': min_x, 'max_x': max_x,
                        'min_y': min_y, 'max_y': max_y,
                        'cx': (min_x + max_x) / 2.0,
                        'cy': (min_y + max_y) / 2.0
                    })
                    self.get_logger().info(f"Loaded Restricted Zone '{label}': X=[{min_x}, {max_x}], Y=[{min_y}, {max_y}]")
        except Exception as e:
            self.get_logger().error(f"Failed to parse OWL ontology: {e}. Falling back to default layout.")
            # Standard SW yellow-black markings
            zones = [{
                'label': 'Loading Dock SW (yellow-black stripes)',
                'min_x': -12.0, 'max_x': -6.0,
                'min_y': -3.5, 'max_y': 3.5,
                'cx': -9.0, 'cy': 0.0
            }]
        return zones

    def cmd_cb(self, msg: Twist):
        """Monitors commanded velocities and filters/intercepts them."""
        self.last_cmd = msg
        
        # Check semantic restricted zones
        zone_violated, zone = self.check_zone_violation()

        if zone_violated and zone:
            # ESCAPE LOGIC: Allow driving away from the zone center
            is_escaping = self.check_escape_direction(zone, msg)
            if is_escaping:
                self.stopped_by_zone = False
                self.cmd_pub.publish(msg)  # Pass-through escape command
            else:
                self.stopped_by_zone = True
                self.cmd_pub.publish(Twist())  # Emergency Stop
                
                # Throttle terminal warnings
                now = self.get_clock().now()
                if (now - self.last_print).nanoseconds >= 400_000_000:
                    self.last_print = now
                    print(f"\n{RED}{BOLD}{'═'*65}\n"
                          f"  🛑  COLLISION GUARD — RESTRICTED ZONE VIOLATION!\n"
                          f"  Robot attempted to cross yellow-black hazard line!\n"
                          f"{'═'*65}\n"
                          f"  Zone:   {zone['label']}\n"
                          f"  Steer AWAY to back out of the restricted zone.\n"
                          f"{'═'*65}{RESET}")
        else:
            self.stopped_by_zone = False
            # If not stopped by Lidar either, pass the velocity forward
            if not self.stopped_by_lidar:
                self.cmd_pub.publish(msg)

    def check_zone_violation(self):
        """Checks if the robot's real-time TF pose is inside any OWL restricted zone."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y

            margin = self.ROBOT_RADIUS
            for zone in self.restricted_zones:
                if (zone['min_x'] - margin <= rx <= zone['max_x'] + margin) and \
                   (zone['min_y'] - margin <= ry <= zone['max_y'] + margin):
                    return True, zone
        except TransformException:
            pass
        return False, None

    def check_escape_direction(self, zone, cmd: Twist):
        """Allows teleoperation commands that guide the robot AWAY from the keepout center."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            
            # Vector pointing from zone center to robot
            escape_dx = rx - zone['cx']
            escape_dy = ry - zone['cy']

            # Commanded driving vector
            # cmd.linear.x > 0 is forward, < 0 is backward
            # Transform cmd to map orientation
            rot = trans.transform.rotation
            # Compute approximate yaw
            siny_cosp = 2 * (rot.w * rot.z + rot.x * rot.y)
            cosy_cosp = 1 - 2 * (rot.y * rot.y + rot.z * rot.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            vx_map = cmd.linear.x * math.cos(yaw)
            vy_map = cmd.linear.x * math.sin(yaw)

            # Check if velocity points in the same general direction as the escape vector
            dot_product = vx_map * escape_dx + vy_map * escape_dy
            return dot_product > 0.01  # True means driving away from danger center
        except Exception:
            # Fallback: if we can't calculate, allow backward commands (manual backing out)
            return cmd.linear.x < 0.0

    def scan_cb(self, msg: LaserScan):
        """Lidar safety monitoring."""
        n = len(msg.ranges)
        if n == 0:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        sectors = {'FRONT': [], 'LEFT': [], 'RIGHT': [], 'BACK': []}

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                continue
            angle = angle_min + i * angle_inc
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)

            if -45 <= deg <= 45:
                sectors['FRONT'].append(r)
            elif 45 < deg <= 135:
                sectors['LEFT'].append(r)
            elif -135 <= deg < -45:
                sectors['RIGHT'].append(r)
            else:
                sectors['BACK'].append(r)

        min_dists = {name: min(vals) if vals else float('inf') for name, vals in sectors.items()}
        danger_sectors = {name: d for name, d in min_dists.items() if d <= self.STOP_DISTANCE}

        if danger_sectors:
            self.stopped_by_lidar = True
            self.cmd_pub.publish(Twist())  # E-Stop

            now = self.get_clock().now()
            if (now - self.last_print).nanoseconds >= 400_000_000:
                self.last_print = now
                icons = {'FRONT': '⬆️', 'BACK': '⬇️', 'LEFT': '⬅️', 'RIGHT': '➡️'}
                lines = [
                    f'\n{RED}{BOLD}{"═"*55}',
                    f'  🛑  COLLISION GUARD — PHYSICAL OBSTACLE STOP',
                    f'{"═"*55}{RESET}'
                ]
                for name, d in danger_sectors.items():
                    lines.append(f'  {icons[name]}  {RED}{BOLD}{name:6s}  OBSTACLE at {d:.2f} m{RESET}')
                lines.append(f'{RED}{BOLD}{"═"*55}{RESET}')
                print('\n'.join(lines))
        else:
            if self.stopped_by_lidar:
                self.stopped_by_lidar = False
                self.get_logger().info(f'{GREEN}Obstacle path clear — releasing e-stop{RESET}')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
