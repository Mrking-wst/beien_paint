from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_plc_communicate_cpp',
            executable='plc_communicate',
            name='plc_communicate',
            output='screen',
            parameters=['config/plc_params.yaml']
        )
    ])