#!/usr/bin/env python3
"""MiR100 AMR Navigation Launch File (No Arm)"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 1. 터미널 명령어 인자(Arguments) 설정
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 패키지 경로 가져오기
    gazebo_dir = get_package_share_directory('mobile_manipulator_gazebo')
    nav_dir = get_package_share_directory('mobile_manipulator_nav')

    # 2. 가제보 시뮬레이션 환경 실행 (MiR100 스폰 포함)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'custom_simulation.launch.py')
        ),
        launch_arguments={
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 3. Nav2 (자율주행 두뇌) 및 RViz2 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items()
    )

    # 💡 팁: 가제보가 완전히 켜지고 센서가 작동할 시간을 벌기 위해 Nav2를 10초 늦게 켭니다.
    delayed_nav2_launch = TimerAction(
        period=10.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        # 실행 인자 선언부
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (no GUI)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),

        # 시스템 구동
        gazebo_launch,
        delayed_nav2_launch,
    ])