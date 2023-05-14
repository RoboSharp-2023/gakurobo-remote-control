from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='joy',
                executable='joy_node',
                name='joy'
            ),
            
            Node(
                package='float_to_uint8array',
                executable='main',
                exec_name='float_to_uint8array',
                name='float_to_uint8array'
            ),
            
            Node(
                package='spd_to_omni',
                executable='main',
                exec_name='spd_to_omni',
                name='spd_to_omni'
            ),
            
            Node(
                package='controller_node',
                executable='main',
                exec_name='controller_node',
                name='controller_node',
                output='screen'
            ),
            
            Node(
                package='integrate_message',
                executable='main',
                exec_name='integrate_msg',
                name='integrate_msg',
            ),
        ]
    )