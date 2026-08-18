import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_warehouse_pkg')
    
    # Xacro 파일 경로
    xacro_file = os.path.join(pkg_share, 'models', 'fixed_ur5e_vacuum.urdf.xacro')
    
    # Xacro 파싱
    robot_description_raw = Command(['xacro ', xacro_file])
    
    # 스폰 위치 설정 (스토퍼 옆: X=2.5, Y=0.75, Z=0.0)
    x_pose = LaunchConfiguration('x_pose', default='2.5')
    y_pose = LaunchConfiguration('y_pose', default='0.75')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    # Robot State Publisher 노드
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Gazebo 스폰 노드 (ros_gz_sim)
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_raw,
            '-name', 'fixed_ur5e_arm',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '-1.5707'  # 컨베이어 쪽을 바라보도록 90도 회전
        ]
    )
    
    # 관절 상태 센서 켜기
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 팔 모터(궤적 제어기) 켜기
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_arm_controller
    ])