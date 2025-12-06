from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'online_async_launch.py')),
    #     launch_arguments={'slam_params_file': '/home/sccoutmini/ros2_hum/src/scout_bringup/config/mapper_params_online_async.yaml', 'use_sim_time': 'true'}.items()
    # )

    # teleop = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_node',
    #     remappings=[
    #         ('cmd_vel', 'cmd_vel_key'),
    #     ]
    # )

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
        twist_mux
    ])