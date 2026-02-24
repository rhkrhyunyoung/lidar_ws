import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 경로 설정
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')
    
    # URDF 파일 경로 (사용자님 시스템의 절대 경로)
    urdf_file_path = '/home/rhkrgusdud/lidar_ws/src/FAST_LIO/urdf/DROK_P.urdf'
    
    # 2. URDF 파일 읽기 (들여쓰기 주의)
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. 인자 정의
    map_path = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/mnt/data/map_result/my_fast_lio_map4.pcd',
        description='Full path to the PCD map file'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )

    # 4. 노드 설정
    # FAST-LIO 노드
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[
            os.path.join(default_config_path, 'velodyne_localization.yaml'),
            {'pcd_path': map_path, 'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Robot State Publisher (로봇 모델 표시)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Joint State Publisher (관절 에러 해결용 - 반드시 추가!)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Static TF (body -> base_link 연결)
    # 중요: FAST-LIO가 쏘는 'body'가 부모가 되어야 차체가 따라갑니다.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link']
    )

    # RViz2 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_use_sim_time_cmd,
        fast_lio_node,
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_tf_node 
    ])
