from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    #run lidar
    rslidar_driver = Node(
        package = 'rslidar_sdk',
        executable = 'rslidar_sdk_node',
        name = 'rslidar_sdk_node',
        output = 'screen'
    )

    pointcloud_to_laserscan = Node(
        package = 'pointcloud_to_laserscan',
        executable = 'pointcloud_to_laserscan_node',
        name = 'pointcloud_to_laserscan_node',
        remappings = [
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'rslidar',
            'min_height': -0.5,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'range_min': 0.1,
            'range_max': 60.0,
        }],
    )

    return LaunchDescription([
        rslidar_driver,
        pointcloud_to_laserscan,
    ])