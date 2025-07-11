cmake_minimum_required(VERSION 3.8)
project(io_mocap)

# 默认依赖（ROS 2 必须）
find_package(ament_cmake REQUIRED)

# 安装 Python Launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 URDF 和 RViz 配置文件
install(DIRECTORY description rviz
  DESTINATION share/${PROJECT_NAME}
)

# 声明 Python 模块（如果有 Python 代码）
# ament_python_install_package(${PROJECT_NAME})

# 导出依赖
ament_export_dependencies(
  rclpy
  robot_state_publisher
  joint_state_publisher_gui
  rviz2
)

ament_package()