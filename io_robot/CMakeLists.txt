cmake_minimum_required(VERSION 3.8)
project(io_robot)

# 默认依赖（ROS 2 必须）
find_package(ament_cmake REQUIRED)

# 安装 Python Launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 声明 Python 模块（如果有 Python 代码）
install(PROGRAMS
  src/robot_sim_ros2.py
  src/robot_sim_bullet.py
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclpy
  robot_state_publisher
  joint_state_publisher_gui
  rviz2
)

ament_package()