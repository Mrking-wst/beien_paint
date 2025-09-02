from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node,PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('pkg_plc_communicate_cpp'),
        'config',
        'plc_params.yaml'
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('plc'),    # 推送命名空间
                Node(
                    package='pkg_plc_communicate_cpp',
                    executable='plc_communicate',
                    name='plc_communicate',
                    output='screen',
                    parameters=[config]
                )
        ])
    ])