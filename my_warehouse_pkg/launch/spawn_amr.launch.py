import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    mir_desc_dir = get_package_share_directory('mir_description')
    mir_share_parent = os.path.dirname(mir_desc_dir)
    
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        if mir_share_parent not in os.environ['GZ_SIM_RESOURCE_PATH']:
            os.environ['GZ_SIM_RESOURCE_PATH'] += f":{mir_share_parent}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = mir_share_parent

    desc_dir = get_package_share_directory('mobile_manipulator_description')
    xacro_file = os.path.join(desc_dir, 'urdf', 'custom_amr.gazebo.xacro')
    
    robot_desc_command = Command(['xacro ', xacro_file])
    robot_desc_param = ParameterValue(robot_desc_command, value_type=str)

    # 💡 핵심 수정 1: amr 네임스페이스에 갇힌 TF를 글로벌(/tf)로 꺼내줍니다!
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='amr_state_publisher',
        namespace='amr',
        output='screen',
        parameters=[{'robot_description': robot_desc_param, 'use_sim_time': True}],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )

    spawn_amr = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc_command,
            '-name', 'custom_mir100',
            '-x', '0.0',
            '-y', '2.0',
            '-z', '0.1'
        ]
    )

    # 💡 핵심 수정 2: Nav2가 찾는 표준 이름인 /odom 으로 완벽하게 연결해 줍니다!
    amr_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel_gz@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom_gz@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf_gz@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/tf_gz', '/tf'),
            ('/odom_gz', '/odom'), 
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_amr,
        amr_bridge
    ])