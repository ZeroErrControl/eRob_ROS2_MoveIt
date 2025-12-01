cmake_minimum_required(VERSION 3.8)
project(erob_description)

find_package(ament_cmake REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  scripts/robot_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装目录
install(DIRECTORY
  launch
  meshes
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

