from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node,PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('pkg_beien_paint_cpp'),
        'config',
        'plc_mapping.yaml'
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('plc/manager'),    # 推送命名空间
                Node(
                    package='pkg_beien_paint_cpp',
                    executable='plc_manager',
                    name='plc_manager',
                    output='screen',
                    parameters=[config]
                )
        ])
    ])