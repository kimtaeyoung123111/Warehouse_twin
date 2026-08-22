import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_dir = get_package_share_directory('mobile_manipulator_description')
    xacro_path = os.path.join(desc_dir, 'urdf', 'fixed_ur5e_vacuum.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='arm',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/arm/robot_description',
            '-name', 'fixed_ur5e',
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # =============== [여기서부터 새로 추가!] ===============
    # Gazebo의 /vacuum_gripper/command 토픽을 ROS 2로 넘겨주는 브릿지
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/vacuum_gripper/command@std_msgs/msg/Bool]gz.msgs.Boolean'
        ],
        output='screen'
    )
    # =======================================================

    # 마지막 배열에 bridge_node 추가
    return LaunchDescription([rsp_node, spawn_node, bridge_node])