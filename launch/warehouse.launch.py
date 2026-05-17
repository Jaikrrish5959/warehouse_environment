import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_env')
    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    
    # Declare launch arguments
    headless = LaunchConfiguration('headless')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (server only)'
    )

    # Set Gazebo resource path
    models_path = os.path.join(pkg_share, 'models')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path]
    )

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Conditionally configure Gazebo arguments based on headless parameter
    gz_args = PythonExpression([
        "'-s -r ' + '", world_file, "' if '", headless, "' == 'true' else '-r ' + '", world_file, "'"
    ])

    # Gazebo sim launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'articubot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # ROS-GZ Bridge for clock + lidar
    bridge_clock_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/scan', '/scan_raw'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ROS-GZ Bridge for camera (separate because GZ topic name differs)
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Scan frame relay — rewrites Gazebo's frame_id to match URDF TF tree
    scan_relay = Node(
        package='warehouse_env',
        executable='scan_frame_relay.py',
        name='scan_frame_relay',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Controller Spawners — delayed to let Gazebo fully initialize
    diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
                output='screen',
            )
        ]
    )

    joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
                output='screen',
            )
        ]
    )

    # RViz node
    rviz_config_file = os.path.join(pkg_share, 'config', 'warehouse.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(headless),
        output='screen'
    )

    return LaunchDescription([
        declare_headless,
        gz_resource_path,
        rsp,
        gz_sim,
        spawn_robot,
        bridge_clock_lidar,
        bridge_camera,
        scan_relay,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz
    ])
