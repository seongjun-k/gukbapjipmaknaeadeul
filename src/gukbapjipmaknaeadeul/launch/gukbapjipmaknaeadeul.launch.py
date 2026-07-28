import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('gukbapjipmaknaeadeul')
    # Nav2는 노트북에서 실행 — pinky는 bringup(모터·라이다·odom/tf)만 담당
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': os.path.join(share, 'config', 'gukbab_map.yaml'),
            'params_file': os.path.join(share, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
        }.items(),
    )
    return LaunchDescription([
        nav2,
        Node(
            package='gukbapjipmaknaeadeul',
            executable='orchestrator',
            name='orchestrator',
            output='screen',
        ),
    ])
