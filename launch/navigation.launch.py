import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_env')
    
    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox Launch
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params
        }.items()
    )

    # Nav2 Bringup Launch
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        slam_launch,
        nav2_launch
    ])
