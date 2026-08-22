import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    mobile_manipulator_gazebo_dir = get_package_share_directory('mobile_manipulator_gazebo')
    mobile_manipulator_desc_dir = get_package_share_directory('mobile_manipulator_description')
    mir_description_dir = get_package_share_directory('mir_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # 💡 수정 1: 내 커스텀 창고 맵 및 모델 절대 경로 지정
    # (실제 jetty.sdf 파일과 models 폴더가 있는 경로에 맞게 적절히 수정해 주세요)
    my_world_path = '/home/taeyoung/warehouse_ws/src/my_warehouse_pkg/worlds/jetty.sdf'
    my_models_path = '/home/taeyoung/warehouse_ws/src/my_warehouse_pkg/models'

    # Launch configurations (💡 수정 2: 맵 충돌을 피하기 위해 안전한 중앙 공터로 좌표 수정)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='2.0')
    z_pose = LaunchConfiguration('z_pose', default='0.15')

    # 💡 수정 3: GZ_SIM_RESOURCE_PATH에 내 창고 모델 경로(my_models_path) 추가
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            my_models_path,
            os.path.join(mobile_manipulator_gazebo_dir, '..'),
            os.path.join(mobile_manipulator_desc_dir, '..'),
            os.path.join(mir_description_dir, '..'),
        ])
    )

    # 💡 수정 4: 복잡한 PathJoinSubstitution 대신 내가 지정한 창고 맵 경로(my_world_path)로 직행!
    world_path = my_world_path

    gz_gui_config_path = PathJoinSubstitution([
        mobile_manipulator_gazebo_dir,
        'config',
        'gazebo_gui.config'
    ])

    gz_args_prefix = PythonExpression([
        "'-r -s' if '", LaunchConfiguration('headless', default='false'), "' == 'true' else '-r'"
    ])

    # Process Xacro -> robot_description
    xacro_path = os.path.join(
        mobile_manipulator_desc_dir,
        'urdf',
        'custom_amr.gazebo.xacro'
    )
    robot_desc = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Gazebo Harmonic Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                gz_args_prefix,
                ' ',
                world_path,
                ' ',
                '--gui-config ',
                gz_gui_config_path,
            ],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Spawn Mobile Manipulator Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_mobile_manipulator',
        output='screen',
        arguments=[
            '-name', 'mobile_manipulator',
            '-topic', '/robot_description',
            ['-x=', x_pose],
            ['-y=', y_pose],
            ['-z=', z_pose],
        ]
    )

    # Clock Bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # cmd_vel bridge: ROS 2 /cmd_vel  --->  Gazebo /cmd_vel_gz
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/cmd_vel_gz@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[
            ('/cmd_vel_gz', '/cmd_vel'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # odom + tf bridge: Gazebo  --->  ROS 2
    odom_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_tf_bridge',
        output='screen',
        arguments=[
            '/odom_gz@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf_gz@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/odom_gz', '/odom'),
            ('/tf_gz', '/tf'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Sequence Spawners
    # load_joint_state_broadcaster = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless mode'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Spawn x position'),
        DeclareLaunchArgument('y_pose', default_value='2.0', description='Spawn y position'),
        DeclareLaunchArgument('z_pose', default_value='0.15', description='Spawn z position'),

        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        clock_bridge,
        cmd_vel_bridge,
        odom_tf_bridge,
        # load_joint_state_broadcaster,
    ])