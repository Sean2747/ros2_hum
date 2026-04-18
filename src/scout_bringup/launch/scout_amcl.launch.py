from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
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

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'map_subscribe_transient_local':'true'}.items()
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
        launch_arguments={'map' : '/home/sccoutmini/ros2_hum/src/scout_bringup/maps/mapOneSave.yaml', 'use_sim_time' : 'false'}.items()
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_node',
        remappings=[
            ('/cmd_vel_out', '/cmd_vel'),
        ],
        parameters=[{
            'params_file': '/home/sccoutmini/ros2_hum/src/scout_bringup/config/twist_mux.yaml'
        }]
    )

    return LaunchDescription([
        base_description,
        general_description,
        scout_lidar,
        nav2_launch,
        amcl_launch,
        twist_mux
    ])
