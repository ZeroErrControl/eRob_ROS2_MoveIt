from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import xml.etree.ElementTree as ET

def get_robot_models(pkg_path):
    """获取所有可用的机器人模型"""
    urdf_dir = os.path.join(pkg_path, 'urdf')
    if not os.path.exists(urdf_dir):
        return []

    models = []
    # 需要排除的特殊模型
    excluded_models = ['multi_erobo']
    
    for model_dir in os.listdir(urdf_dir):
        # 跳过被排除的模型
        if model_dir in excluded_models:
            continue
            
        model_path = os.path.join(urdf_dir, model_dir)
        urdf_file = os.path.join(model_path, f"{model_dir}.urdf")
        if os.path.isdir(model_path) and os.path.exists(urdf_file):
            models.append(model_dir)
    return models

def launch_setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('erob_description')
    models = get_robot_models(pkg_path)
    
    print(f"原始模型列表: {models}")
    
    # 检查是否有重复的模型
    if len(models) != len(set(models)):
        print(f"⚠️ 警告：检测到重复的模型名称！重复项: {[m for m in models if models.count(m) > 1]}")
        # 移除重复项
        models = list(set(models))
    
    # 按照模型大小排序
    sorted_models = sort_models_by_size(models)
    
    print(f"排序后的模型列表: {sorted_models}")
    
    # 计算模型位置
    positions = calculate_model_positions(sorted_models)
    print(f"模型位置: {positions}")
    
    print(f"✅ 找到以下机器人模型: {', '.join(sorted_models)}")
    
    # 创建节点列表
    nodes = []
    
    # 创建一个合并的URDF文件，包含所有机器人模型
    merged_urdf = create_merged_urdf(pkg_path, sorted_models)
    
    # 添加世界坐标系
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
            output='screen',
        )
    )
    
    # 移除joint_state_publisher节点，完全依赖外部程序控制关节
    
    # 创建一个临时脚本来过滤关节状态
    joint_state_filter_script = """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class JointStateFilter(Node):
    def __init__(self):
        super().__init__('joint_state_filter')
        
        # 创建QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅原始关节状态
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # 发布过滤后的关节状态
        self.publisher = self.create_publisher(
            JointState,
            '/filtered_joint_states',
            qos)
        
        # 存储最后一次有效的关节状态
        self.last_valid_state = None
        
        self.get_logger().info('Joint state filter started')
    
    def joint_state_callback(self, msg):
        # 检查是否所有位置都是0
        all_zeros = all(abs(pos) < 0.0001 for pos in msg.position)
        
        if not all_zeros:
            # 如果不是全零，则更新最后一次有效状态并发布
            self.last_valid_state = msg
            self.publisher.publish(msg)
        elif self.last_valid_state is not None:
            # 如果是全零但有之前的有效状态，则发布之前的状态
            # 更新时间戳
            self.last_valid_state.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.last_valid_state)

def main():
    rclpy.init()
    node = JointStateFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

    temp_filter_script = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.py')
    temp_filter_script.write(joint_state_filter_script)
    temp_filter_script.close()
    os.chmod(temp_filter_script.name, 0o755)  # 使脚本可执行

    # 添加关节状态过滤节点
    nodes.append(
        Node(
            package='erob_description',
            executable=temp_filter_script.name,
            name='joint_state_filter',
            output='screen',
        )
    )

    # 修改robot_state_publisher使用过滤后的话题，并发布到自定义话题
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': merged_urdf,
                    'publish_frequency': 100.0
                }
            ],
            remappings=[
                ('/joint_states', '/filtered_joint_states'),
                ('/robot_description', '/erob/robot_description')  # 修改机器人描述话题
            ],
            output='screen',
        )
    )
    
    # 使用固定的RViz配置文件
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'ZeroERR_eRob.rviz')
    
    # 如果配置文件不存在，创建一个
    if not os.path.exists(rviz_config_path):
        # 确保rviz目录存在
        os.makedirs(os.path.dirname(rviz_config_path), exist_ok=True)
        
        # 创建一个基本的RViz配置，修改机器人描述话题
        with open(rviz_config_path, 'w') as f:
            f.write("""
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Alpha: 0.4
      Cell Size: 0.3
      Normal Cell Count: 0
      Plane: XY
      Line Style: Lines
      Reference Frame: world

    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Axes: true
      Show Arrows: true
      Show Names: false
      Marker Scale: 0.1
      Frame Timeout: 10.0

    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /erob/robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
      Links:
        All Links Enabled: true

  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 100
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Name: Current View
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398
""")
    
    # 添加RViz节点
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        )
    )
    
    return nodes

def sort_models_by_size(models):
    """按照模型大小排序，并将i型和t型分开"""
    # 首先按照类型分组（i型和t型）
    i_type_models = [model for model in models if 'iv' in model or ('i' in model and 't' not in model)]  # i型模组包含"iv"或只有"i"但不含"t"
    t_type_models = [model for model in models if 't' in model]   # t型模组包含"t"标记
    
    # 提取模型名称中的数字部分
    def extract_number(model_name):
        # 从模型名称中提取数字
        import re
        match = re.search(r'erob(\d+)', model_name)
        if match:
            return int(match.group(1))
        return 0
    
    # 分别对i型和t型模组按照提取的数字排序
    sorted_i_type = sorted(i_type_models, key=extract_number)
    sorted_t_type = sorted(t_type_models, key=extract_number)
    
    # 先返回i型模组，再返回t型模组
    return sorted_i_type + sorted_t_type

def calculate_model_positions(models):
    """计算每个模型的位置，将I型和T型分开排列"""
    positions = {}
    
    # 提取模型名称中的数字部分
    def extract_number(model_name):
        import re
        match = re.search(r'erob(\d+)', model_name)
        if match:
            return int(match.group(1))
        return 0
    
    # 获取每个模型的大小和类型
    model_sizes = {model: extract_number(model) for model in models}
    model_types = {model: 'i' if ('iv' in model or ('i' in model and 't' not in model)) else 't' for model in models}
    
    # 分离I型和T型模型
    i_type_models = [model for model in models if model_types[model] == 'i']
    t_type_models = [model for model in models if model_types[model] == 't']
    
    # I型模型从小到大排序
    i_type_models.sort(key=extract_number)
    
    # T型模型从大到小排序
    t_type_models.sort(key=lambda x: -extract_number(x))
    
    # 计算I型模型的位置（从左到右）
    current_position = -0.5  # 从-1开始，给I型模型留出足够空间
    for model in i_type_models:
        size = model_sizes[model]
        spacing = size * 0.0012  # 使用较小的系数使模型更紧凑
        positions[model] = current_position
        current_position += spacing
    
    # 在I型和T型之间留出间隔
    if i_type_models and t_type_models:
        current_position += 0
          # 额外间隔
    
    # 计算T型模型的位置（从左到右）
    for model in t_type_models:
        size = model_sizes[model]
        spacing = size * 0.0012  # 使用较小的系数使模型更紧凑
        positions[model] = current_position
        current_position += spacing
    
    return positions

def create_merged_urdf(pkg_path, models):
    """创建一个合并的URDF文件，包含所有机器人模型"""
    # 创建一个新的URDF根元素
    root = ET.Element('robot', name='merged_robots')
    
    # 计算模型位置
    positions = calculate_model_positions(models)
    
    # 处理每个模型
    for model in models:
        print(f"处理模型 {model}，URDF文件: {os.path.join(pkg_path, 'urdf', model, f'{model}.urdf')}")
        urdf_file = os.path.join(pkg_path, 'urdf', model, f'{model}.urdf')
        
        # 解析URDF文件
        model_tree = ET.parse(urdf_file)
        model_root = model_tree.getroot()
        
        # 获取模型的Y位置
        y_offset = positions[model]
        
        # 创建一个固定链接，连接到世界坐标系
        world_link = ET.SubElement(root, 'link', name=f'{model}_world')
        
        # 创建一个固定关节，连接到世界坐标系
        world_joint = ET.SubElement(root, 'joint', name=f'{model}_to_world', type='fixed')
        ET.SubElement(world_joint, 'parent', link='world')
        ET.SubElement(world_joint, 'child', link=f'{model}_world')
        
        # 修改为Y方向偏移，并添加绕X轴逆时针旋转90度 (π/2)，同时向X方向移动1个单位
        # 在URDF中，rpy表示roll-pitch-yaw，分别对应绕x、y、z轴的旋转
        # 绕x轴逆时针旋转90度，对应roll值为π/2
        # 添加Z轴偏移0.5，使所有模型向上移动
        origin = ET.SubElement(world_joint, 'origin', xyz=f'1.5 {y_offset} 0.3', rpy='1.5708 0 0')
        
        # 添加模型的所有元素
        for child in model_root:
            # 修改链接和关节的名称，添加模型前缀
            if child.tag in ['link', 'joint']:
                name = child.get('name')
                child.set('name', f'{model}_{name}')
                
                # 如果是关节，修改父链接和子链接的名称
                if child.tag == 'joint':
                    parent = child.find('parent')
                    child_link = child.find('child')
                    
                    if parent is not None:
                        parent_link = parent.get('link')
                        parent.set('link', f'{model}_{parent_link}')
                    
                    if child_link is not None:
                        child_name = child_link.get('link')
                        child_link.set('link', f'{model}_{child_name}')
            
            # 添加到根元素
            root.append(child)
        
        # 创建一个固定关节，连接世界链接到模型的基础链接
        base_joint = ET.SubElement(root, 'joint', name=f'{model}_base_joint', type='fixed')
        ET.SubElement(base_joint, 'parent', link=f'{model}_world')
        ET.SubElement(base_joint, 'child', link=f'{model}_base_link')
        origin = ET.SubElement(base_joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    
    # 创建一个世界链接
    world_link = ET.SubElement(root, 'link', name='world')
    
    # 将XML树转换为字符串
    return ET.tostring(root, encoding='unicode')

def create_joint_angle_display_script(models):
    """创建一个Python脚本，用于显示关节角度"""
    script = """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
import math
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

class JointAngleDisplay(Node):
    def __init__(self):
        super().__init__('joint_angle_display')
        
        # 创建一个订阅者，订阅关节状态话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # 创建一个发布者，发布标记数组
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/joint_angles',
            10)
        
        # 存储关节名称和角度
        self.joint_angles = {}
        
        # 存储每个模型的最新角度
        self.model_angles = {}
        
        # 创建TF缓冲区和监听器，用于获取旋转轴的位置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 设置定时器，定期更新标记
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('Joint angle display node started')
    
    def joint_state_callback(self, msg):
        # 更新关节角度
        for i, name in enumerate(msg.name):
            if 'revolute' in name:  # 只关注旋转关节
                self.joint_angles[name] = msg.position[i]
                
                # 提取模型名称
                parts = name.split('_')
                if len(parts) >= 2:
                    model_name = parts[0]
                    # 更新该模型的最新角度
                    self.model_angles[model_name] = msg.position[i]
    
    def publish_markers(self):
        if not self.model_angles:
            return
        
        marker_array = MarkerArray()
        
        # 为每个模型创建一个文本标记
        for i, (model_name, angle) in enumerate(self.model_angles.items()):
            # 创建标记
            marker = Marker()
            
            # 使用模型的revolute_link作为参考框架，这是旋转轴所在的链接
            revolute_frame = f"{model_name}_revolute_link"
            marker.header.frame_id = revolute_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "joint_angles"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # 设置文字位置，直接在旋转轴上
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.05  # 稍微向前偏移，使文字更容易看到
            marker.pose.position.z = 0.0   # 与旋转轴在同一高度
            
            # 设置方向（默认）
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # 统一文本大小
            marker.scale.z = 0.035
            
            # 设置颜色
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            # 设置文本（角度，转换为度）
            degrees = math.degrees(angle)
            marker.text = f"{degrees:.1f}°"
            
            # 设置持续时间（0表示永久显示）
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            
            marker_array.markers.append(marker)
        
        # 发布标记数组
        self.marker_publisher.publish(marker_array)

def main():
    rclpy.init()
    node = JointAngleDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
    return script

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
