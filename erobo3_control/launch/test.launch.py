# demo.launch.py

from functools import partial
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def set_cpu_affinity(context, moveit_config, *args, **kwargs):
    # 生成基础的 LaunchDescription
    ld = generate_demo_launch(moveit_config)
    
    # 确认 LaunchDescription 有效
    if ld is None:
        print("Error: generate_demo_launch returned None.")
        return []
    
    if not hasattr(ld, 'entities') or ld.entities is None:
        print("Error: LaunchDescription has no 'entities' or 'entities' is None.")
        return []
    
    
        # 打印所有节点的属性以调试
    print("LaunchDescription entities:")
    for entity in ld.entities:
        if isinstance(entity, Node):
            print(f" - Node object: {entity}")
            print(f" - Node attributes: {vars(entity)}")  # 打印所有属性
            print(f" - Node dir: {dir(entity)}")  # 打印对象的所有方法和属性
            print(f" - Node name: {entity.name}")  # 尝试打印节点名称
    

    # 为每个节点绑定 CPU 核心
    cpu_assignments = {
        'move_group': '3',               
        'robot_state_publisher': '4',    
        'controller_manager': '5',       
    }
    
    # 遍历节点并设置 CPU 亲和性
    for entity in ld.entities:
        if isinstance(entity, Node):
            node_name = entity.name
            if node_name is None:
                continue
            for target_node, cpu_core in cpu_assignments.items():
                if target_node in node_name:
                    # 使用 sudo schedtool 设置 CPU 亲和性
                    entity.prefix = f'sudo schedtool -a {cpu_core} -F -p 80'
                    print(f" - Setting CPU affinity for node '{node_name}' to CPU {cpu_core}")
                    break
    
    return ld.entities


def generate_launch_description():
    # 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        "eRobo3",
        package_name="erobo3_control"
    ).to_moveit_configs()
    
    # 使用 partial 将 moveit_config 传递给 set_cpu_affinity
    return LaunchDescription([
        OpaqueFunction(function=partial(set_cpu_affinity, moveit_config=moveit_config))
    ])
