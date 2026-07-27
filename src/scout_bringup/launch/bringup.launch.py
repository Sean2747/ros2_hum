from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    map_path = os.getenv("MAP_PATH")

    pose_tracker = Node(
        package = 'scout_bringup',
        executable = 'pose_tracker',
        name = 'pose_tracker',
        output = 'screen',
        parameters=[
            {
                'map_path': map_path
            }
        ]
    )

    map_reader = Node(
        package = 'scout_bringup',
        executable = 'map_reader',
        name = 'map_reader',
        output = 'screen'
    )

    return LaunchDescription([
        pose_tracker,
        map_reader
    ])