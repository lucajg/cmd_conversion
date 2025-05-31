from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_conversion',
            executable='diff_to_ack',
            name='converter'
        )
    ])
