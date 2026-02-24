import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 설정
    pkg_name = 'DROK_P'
    pkg_share = get_package_share_directory(pkg_name)

    # 2. 경로 설정 (URDF 및 RViz 설정 파일)
    urdf_file = os.path.join(pkg_share, 'urdf', 'DROK_P.urdf')
    rviz_config_file = os.path.join(pkg_share, 'urdf.rviz')

    # 3. URDF 파일 읽기
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 로봇 상태 퍼블리셔 (robot_description 파라미터 전달)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 조인트 상태 퍼블리셔 GUI (관절 움직임 제어)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2 실행 (설정 파일 로드)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
