from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            name='controller_main',
            parameters=[
                {'Kp0': 0.0},
                {'Ki0': 0.0},
                {'Kd0': 0.0},
                {'Kp1': 6.0}, #6
                {'Ki1': 0.0},
                {'Kd1': 0.01},  #4.5
                {'Kp2': 10.5},                
                {'Ki2': 0.0},
                {'Kd2': 0.5},
                {'Kp3': 0.0},
                {'Ki3': 0.0},
                {'Kd3': 0.0}
            ]
        ),
    ])
    
