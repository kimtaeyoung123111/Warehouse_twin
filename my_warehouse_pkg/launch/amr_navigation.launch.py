import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # 1. 깃허브 원본 패키지 및 순정 Nav2 패키지 경로
    nav_pkg_dir = get_package_share_directory('mobile_manipulator_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 2. 맵 파일과 깃허브 원본의 '파라미터' 및 'RViz' 설정 파일 경로
    map_file = '/home/taeyoung/warehouse_ws/src/my_warehouse_pkg/maps/jetty_nav_map.yaml'
    nav2_params_file = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(nav_pkg_dir, 'rviz', 'nav.rviz')

    # 💡 3. 핵심 수정: 깃허브의 런치 파일(올인원) 대신 ROS 2 순정 Nav2를 켜서 
    # 두 번째 가제보가 켜지는 것을 막고, 깃허브의 '파라미터'만 강제 주입합니다!
    nav2_group = GroupAction([
        SetRemap(src='/robot_description', dst='/amr/robot_description'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'True',
                'params_file': nav2_params_file # 👉 깃허브 원본의 두뇌 세팅 이식!
            }.items()
        )
    ])

    # 4. 원본 깃허브의 RViz 세팅 그대로 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/robot_description', '/amr/robot_description')
        ]
    )

    return LaunchDescription([
        nav2_group,
        rviz_node
    ])