cmake_minimum_required(VERSION 3.8)
project(lvio_2d)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-O3 -g)
add_definitions(-DNDEBUG)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==========================================
# 1. 查找依赖项 (ROS2 核心包)
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# utils/visualization.cpp 中用到的所有 ROS 2 包都必须找出来
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ==========================================
# 2. 查找第三方非 ROS 包
# ==========================================
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

# ==========================================
# 3. 包含路径
# ==========================================
include_directories(
  src
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ==========================================
# 4. 创建库: utilies
# ==========================================
add_library(utilies SHARED
  src/utilies/common.cpp 
  src/utilies/params.cpp
  src/utilies/visualization.cpp
)

# 【关键修复】将 utilies 用到的所有 ROS 2 包都加进来，否则找不到头文件
ament_target_dependencies(utilies 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport 
  tf2_ros 
  nav_msgs 
  geometry_msgs 
  visualization_msgs
)
# 第三方库
target_link_libraries(utilies ${OpenCV_LIBS})


# ==========================================
# 5. 创建库: trajectory
# ==========================================
add_library(trajectory SHARED
  src/trajectory/trajectory.cpp
  src/trajectory/camera_manager.cpp
  src/trajectory/laser_manager.cpp
  src/trajectory/keyframe_manager.cpp
  src/factor/solver.cpp
)
# 假设 trajectory 库也用到了这些，这里给它补齐基本的
ament_target_dependencies(trajectory 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2_ros
)
target_link_libraries(trajectory ${OpenCV_LIBS} ${CERES_LIBRARIES})


# ==========================================
# 6. 可执行文件
# ==========================================
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# 链接 ROS 2 依赖
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  tf2_ros
  nav_msgs
  geometry_msgs
  visualization_msgs
)

# 链接自己的库和第三方库
target_link_libraries(${PROJECT_NAME}_node 
  utilies
  trajectory
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

# ==========================================
# 7. 安装
# ==========================================
# # 安装二进制目标文件
# install(TARGETS
#   ${PROJECT_NAME}_node
#   utilies
#   trajectory
#   DESTINATION lib/${PROJECT_NAME}
# )

# # 【新增】安装 launch 和 config 目录到 ROS 2 的共享目录下
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )


# 1. 节点可执行文件安装到 lib/包名 下 (ROS 2 规范)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 动态链接库 (.so) 必须直接安装到 lib/ 目录下，系统才能找到！
install(TARGETS utilies trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 3. 安装 launch 和 config 目录
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 8. 导出 ament 环境
# ==========================================
ament_package()