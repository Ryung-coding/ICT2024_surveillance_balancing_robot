from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    bag_directory = os.path.expanduser("~/Desktop/bag")

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
            )
        ]
    )
