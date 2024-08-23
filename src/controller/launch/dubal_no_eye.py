from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='serial_reciver',
                executable='serial_reciver_node',
                name='serial_reciver_node'
            ),
            Node(
                package='odrive_controller',
                executable='odrive_controller',
                name='odrive_controller'
            ),
            Node(
                package='controller',
                executable='controller_main',
                name='controller_main'
            ),
        ]
    )
