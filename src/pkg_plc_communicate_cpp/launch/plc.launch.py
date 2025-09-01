from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node,PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('plc'),    # 推送命名空间
                Node(
                    package='pkg_plc_communicate_cpp',
                    executable='plc_communicate',
                    name='plc_communicate',
                    output='screen',
                    parameters=['config/plc_params.yaml']
                )
        ])
    ])