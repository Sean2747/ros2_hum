from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    base_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('scout_base'), 'launch', 'scout_base.launch.py')), 
        launch_arguments={'port_name': 'can2', 'is_scout_mini': 'true'}.items()
    )

    general_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('scout_description'), 'launch', 'scout_base_description.launch.py'))
    )

    scout_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('scout_bringup'), 'launch', 'scout_lidar.launch.py'))
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': '/home/sccoutmini/ros2_hum/src/scout_bringup/config/mapper_params_online_async.yaml', 'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        base_description,
        general_description,
        scout_lidar,
        slam_launch
    ])
