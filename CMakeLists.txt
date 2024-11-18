cmake_minimum_required(VERSION 3.8)
project(rigid_body_simulation)

# 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
# 添加可执行文件
add_executable(rigid_body_simulation_node src/rigid_body_node.cpp)
# 链接库
target_link_libraries(rigid_body_simulation_node
  Eigen3::Eigen
  yaml-cpp
)

# 添加依赖
ament_target_dependencies(rigid_body_simulation_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
)

# 安装
install(TARGETS
  rigid_body_simulation_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装URDF文件
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
