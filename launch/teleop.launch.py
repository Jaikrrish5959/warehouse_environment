import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch teleop_twist_keyboard remapped to diff_drive_controller.
    
    Run this in a terminal and use keyboard to control the robot:
      u    i    o
      j    k    l
      m    ,    .
    
    i/k : forward/stop
    j/l : turn left/right
    q/z : increase/decrease speed
    """

    # WASD Teleop publishes Twist to /cmd_vel_unstamped
    teleop_node = Node(
        package='warehouse_env',
        executable='wasd_teleop.py',
        name='wasd_teleop',
        remappings=[
            ('/cmd_vel', '/cmd_vel_unstamped'),
        ],
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen',
        prefix='gnome-terminal --wait --',
    )

    # twist_stamper converts Twist to TwistStamped required by diff_drive_controller
    twist_stamper = Node(
        package='warehouse_env',
        executable='twist_stamper.py',
        name='twist_stamper',
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        teleop_node,
        twist_stamper,
    ])
