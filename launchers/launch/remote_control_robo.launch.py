from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='uart_communicator',
                executable='main',
                exec_name='uart_communicator',
                name='uart_communicator',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"serial_name": "/dev/ttyACM0"},
                ]
            ),
            
            Node(
                package='uart_communicator',
                executable='main',
                exec_name='uart_communicator2',
                name='uart_communicator2',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"serial_name": "/dev/ttyUSB0"},
                    {"uart_topic_name": "solenoid_uart_msg"},
                ]
            ),
            
        ]
    )