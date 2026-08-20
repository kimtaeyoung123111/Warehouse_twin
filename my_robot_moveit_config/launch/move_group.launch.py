# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("fixed_arm_robot", package_name="my_robot_moveit_config").to_moveit_configs()
#     return generate_move_group_launch(moveit_config)

from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fixed_arm_robot", package_name="my_robot_moveit_config").to_moveit_configs()
    
    # 💡 런치 파일 전체에 가상 시간(sim_time)을 사용하도록 강제 주입!
    ld = LaunchDescription()
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(generate_move_group_launch(moveit_config))
    
    return ld