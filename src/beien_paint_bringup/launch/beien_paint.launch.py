from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    # -------------------------------
    # PLC 通信
    # -------------------------------
    plc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pkg_plc_communicate_cpp'),
                'launch',
                'plc.launch.py'
            )
        )
    )

    # -------------------------------
    # 底盘控制节点
    # -------------------------------
    beien_paint_node = Node(
        package='pkg_beien_paint_cpp',
        executable='beien_paint',
        name='beien_paint',
        output='screen'
    )

    # -------------------------------
    # Joycon Left
    # -------------------------------
    joycon_left = Node(
        package='pkg_joystick_cpp',
        executable='joycon_left_pub',
        name='joycon_left_pub',
        output='screen'
    )

    # -------------------------------
    # Joycon Right
    # -------------------------------
    joycon_right = Node(
        package='pkg_joystick_cpp',
        executable='joycon_right_pub',
        name='joycon_right_pub',
        output='screen'
    )

    # return LaunchDescription([
    #     Node(
    #         package='pkg_plc_communicate_cpp',
    #         executable='plc_communicate',
    #         name='plc_communicate',
    #         output='screen',
    #         parameters=['config/beien_paint_params.yaml']
    #     )
    # ])

    return LaunchDescription([
        plc_launch,
        beien_paint_node,
        joycon_left,
        joycon_right
    ])
