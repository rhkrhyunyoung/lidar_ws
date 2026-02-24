# 파일 경로: ~/ros2_ws/start_gv7.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 위에서 만든 설정 파일 경로 지정
    config = '/lidar_ws/gv7_params.yaml'
    return LaunchDescription([
        Node(
            package='microstrain_inertial_driver',
            executable='microstrain_inertial_driver_node',
            name='microstrain_inertial_driver',
            namespace='/',
            output='screen',
            parameters=[config],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
