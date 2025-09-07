from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node,PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('pkg_joystick_cpp'),
        'config',
        'joystick_params.yaml'
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('joystick/joycon'),    # 推送命名空间
                Node(
                    package='pkg_joystick_cpp',
                    executable='joycon_left_pub',
                    name='joycon_left_pub',
                    output='screen',
                    parameters=[config]
                ),
                # Node(
                #     package='pkg_joystick_cpp',
                #     executable='joycon_right',
                #     name='joycon_right',
                #     output='screen',
                #     parameters=[config]
                # )
        ])
    ])