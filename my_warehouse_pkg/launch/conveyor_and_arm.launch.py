import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_warehouse_pkg')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    models_path = os.path.join(pkg_share, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        if models_path not in os.environ['GZ_SIM_RESOURCE_PATH']:
            os.environ['GZ_SIM_RESOURCE_PATH'] += f":{models_path}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path

    # 1. Gazebo 실행 (Empty World + 자동 재생 -r 옵션)
    world_file = os.path.join(pkg_share, 'worlds', 'jetty.sdf')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items()
    )

    # 2. 컨베이어 벨트 SDF 모델 스폰 (X=0.0, Y=0.0, Z=0.0)
    conveyor_sdf_file = os.path.join(pkg_share, 'models', 'conveyor_belt.sdf')
    spawn_conveyor = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', conveyor_sdf_file,
            '-name', 'conveyor_belt',
            '-x', '-8.0',
            '-y', '-20.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '1.5707'
        ]
    )

    # 3. 로봇 팔 Xacro 불러오기 & Robot State Publisher
    xacro_file = os.path.join(pkg_share, 'models', 'fixed_ur5e_vacuum.urdf.xacro')
    robot_description_raw = Command(['xacro ', xacro_file])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    
    # 4. UR5e 로봇 팔 스폰 (컨베이어 스토퍼 옆: X=-8.0, Y=-17.0, Z=0.0, 컨베이어 쪽 바라보게 -90도 회전)
    spawn_fixed_arm = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_raw,
            '-name', 'fixed_ur5e_arm',
            '-x', '-7.2',
            '-y', '-17.8',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            # '-Y', '-1.5707'
            '-Y', '3.14159'
        ]
    )

    # 5. 관절 센서 방송기 (Joint State Broadcaster)
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 6. 팔 모터 제어기 (Joint Trajectory Controller)
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # 로봇이 성공적으로 스폰된 이후에 제어기들이 켜지도록 순서 이벤트 등록
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_fixed_arm,
            on_exit=[
                spawn_joint_state_broadcaster,
                spawn_arm_controller
            ]
        )
    )

    # 👇 2. 가제보 카메라 영상을 ROS 2 토픽으로 변환하는 브리지
    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    return LaunchDescription([
        gazebo_sim,
        spawn_conveyor,
        node_robot_state_publisher,
        spawn_fixed_arm,
        delayed_controllers,
        camera_bridge
    ])