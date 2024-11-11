from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 生成 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("eRobo3", package_name="erobo3_control").to_moveit_configs()

    # 启动 ros2_control 的 controller_manager 节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,  # URDF 机器人描述
            {"robot_controller": "eRobo3_controller"}  # 控制器配置
        ]
    )

    # 返回只包含 controller_manager 的 LaunchDescription
    return LaunchDescription([ros2_control_node])
