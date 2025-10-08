cmake_minimum_required(VERSION 3.8)
project(hikvision_camera)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ROS 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# 设置 MVS SDK 头文件目录
include_directories(/opt/MVS/include)

# 设置 MVS SDK 库文件目录
link_directories(/opt/MVS/lib /opt/MVS/lib/64)

# 查找 MVS SDK 的库文件
find_library(MVS_LIBRARY 
  NAMES MvCameraControl 
  PATHS /opt/MVS/lib /opt/MVS/lib/64 /usr/lib /usr/local/lib
  REQUIRED
)

if(NOT MVS_LIBRARY)
  message(FATAL_ERROR "MvCameraControl library not found. Please install Hikvision MVS SDK.")
endif()

message(STATUS "Found MVS library: ${MVS_LIBRARY}")

# 添加可执行文件
add_executable(hikvision_camera_node src/hikvision_camera.cpp)

# 将 MVS 库和 OpenCV 链接到项目
target_link_libraries(hikvision_camera_node 
  ${MVS_LIBRARY}
  ${OpenCV_LIBRARIES}
)

# 确保添加所有 ROS 依赖项
ament_target_dependencies(hikvision_camera_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# 包含 OpenCV 头文件
target_include_directories(hikvision_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# 安装可执行文件
install(TARGETS
  hikvision_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件 - 明确指定文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 导出包含目录
ament_export_include_directories(include)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()