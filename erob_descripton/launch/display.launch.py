from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    
    # 如果urdf_file参数使用默认值，则根据model自动设置
    if urdf_file == f"{model}.urdf":
        urdf_file = f"{model}.urdf"
        print(f"✅ 自动设置URDF文件: {urdf_file}")

    pkg_path = get_package_share_directory('erob_description')
    urdf_path = os.path.join(pkg_path, 'urdf', model, urdf_file)

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"❌ URDF文件未找到: {urdf_path}")
    else:
        print(f"✅ 使用URDF文件: {urdf_path}")

    # 读取URDF文件内容
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
        
    # 根据选择的机器人型号查找对应的RViz配置文件
    rviz_config_path = os.path.join(pkg_path, 'rviz', f"{model}.rviz")
    rviz_args = []
    
    if os.path.exists(rviz_config_path):
        print(f"✅ 使用RViz配置文件: {rviz_config_path}")
        rviz_args = ['-d', rviz_config_path]
    else:
        print(f"⚠️ 未找到RViz配置文件: {rviz_config_path}，将使用默认配置")
    
    # 创建一个临时的RViz配置文件，如果没有找到现有的配置文件
    if not rviz_args:
        import tempfile
        temp_rviz_config = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.rviz')
        temp_rviz_config.write("""
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_link:
          Value: true
        fixed_link:
          Value: false
        revolute_link:
          Value: true
      Marker Scale: 0.1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        base_link:
          fixed_link:
            revolute_link:
              {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Name: Current View
      Pitch: 0.785398
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398
""")
        temp_rviz_config.close()
        rviz_args = ['-d', temp_rviz_config.name]

    return [
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args,
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='erob80iv6',
            description='Name of the robot model folder (under urdf/)'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='${model}.urdf',
            description='URDF file name under that folder (default: <model>.urdf)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
