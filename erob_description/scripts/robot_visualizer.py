#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
import os
from ament_index_python.packages import get_package_share_directory

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        # 获取参数
        self.declare_parameter('models', '')
        self.declare_parameter('spacing', 2.0)
        
        # 解析模型列表（从逗号分隔的字符串）
        models_str = self.get_parameter('models').value
        self.models = [m.strip() for m in models_str.split(',') if m.strip()]
        self.spacing = self.get_parameter('spacing').value
        
        # 创建发布器
        self.marker_pub = self.create_publisher(MarkerArray, 'robot_models', 10)
        
        # 设置定时器
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info(f'将显示以下机器人模型: {", ".join(self.models)}')
    
    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, model in enumerate(self.models):
            # 为每个模型创建一个标记
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = model
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            
            # 设置位置
            marker.pose.position = Point(x=i * self.spacing, y=0.0, z=0.0)
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # 设置大小
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # 设置颜色
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            # 设置网格资源
            pkg_path = get_package_share_directory('erob_description')
            mesh_path = os.path.join(pkg_path, 'meshes', model, 'base_link.STL')
            if os.path.exists(mesh_path):
                marker.mesh_resource = f"file://{mesh_path}"
            else:
                self.get_logger().warn(f'未找到模型 {model} 的网格文件')
                continue
            
            marker.mesh_use_embedded_materials = True
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 