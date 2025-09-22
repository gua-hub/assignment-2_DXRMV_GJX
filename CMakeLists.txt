# 声明 CMake 版本需求
cmake_minimum_required(VERSION 3.0.0)
# 声明项目与版本/语言等信息
project(opencv_project VERSION 0.1.0 LANGUAGES C CXX)
find_package(OpenCV REQUIRED)
# 将 main.cpp 编译为名为 cpp 的二进制文件 
add_executable(cpp src/main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(cpp ${OpenCV_LIBS})
