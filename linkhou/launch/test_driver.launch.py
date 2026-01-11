from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 获取 URDF 内容
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("linkhou"), "config", "LR4_R560.urdf.xacro"] # ⚠️注意：确认你的xacro文件名是否叫这个
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. 这里的路径要改成实际的 controller 配置文件路径
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("linkhou"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # 3. 单独启动 ros2_control_node，不启动 Rviz 和 MoveIt
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",  # 关键：直接打印到屏幕
    )

    return LaunchDescription([
        control_node,
    ])
