# 声明 CMake 版本需求
cmake_minimum_required(VERSION 3.0.0)

# 声明项目与版本/语言等信息
project(opencv_project VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)

# 将 main.cpp 编译为名为 cpp 的二进制文件 
add_executable(cpp src/main.cpp)
add_executable(cpp1 src/main1.cpp)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS})

#target_include_directories(cpp PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_link_libraries(cpp ${OpenCV_LIBS})
#target_include_directories(cpp1 PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_link_libraries(cpp1 ${OpenCV_LIBS})
target_link_libraries(cpp ${OpenCV_LIBS} ${CERES_LIBRARIES} ${GLOG_LIBRARIES})
target_link_libraries(cpp1 ${OpenCV_LIBS} ${CERES_LIBRARIES} ${GLOG_LIBRARIES})

