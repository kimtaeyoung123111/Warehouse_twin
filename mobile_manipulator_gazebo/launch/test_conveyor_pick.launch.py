import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ★ 핵심 해결: ROS 2의 공유 폴더 최상위 경로를 뽑아냅니다. (예: /opt/ros/jazzy/share)
    ros_share_path = os.path.dirname(get_package_share_directory('ur_description'))

    # 가제보가 3D 피부 파일들을 확실하게 찾을 수 있도록 환경 변수를 강제 주입합니다.
    set_env_action = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[ros_share_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    gazebo_dir = get_package_share_directory('mobile_manipulator_gazebo')
    desc_dir = get_package_share_directory('mobile_manipulator_description')

    xacro_path = os.path.join(desc_dir, 'urdf', 'fixed_ur5e_vacuum.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    world_path = os.path.join(gazebo_dir, 'worlds', 'conveyor_world.sdf')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_path} -r'}.items()
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'fixed_arm',
            '-x', '0.0',
            '-y', '-0.7',
            '-z', '0.0'
        ],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/vacuum_gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/vacuum_gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty'
        ],
        output='screen'
    )

    # set_env_action이 가장 먼저 실행되도록 맨 앞에 배치합니다.
    return LaunchDescription([set_env_action, gazebo_launch, rsp_node, spawn_arm, bridge_node])