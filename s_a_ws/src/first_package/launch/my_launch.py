from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='first_package',
            executable='gui',
        ),        
        Node(
            package='first_package',
            executable='map_subscriber',
        ),
        Node(
            package='first_package',
            executable='map_publisher',
        ),
        Node(
            package='first_package',
            executable='robot',
        ),
        Node(
            package='first_package',
            executable='robot_odometry',
        )
    ])
