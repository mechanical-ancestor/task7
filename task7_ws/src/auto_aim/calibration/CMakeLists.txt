cmake_minimum_required(VERSION 3.10)
project(CameraCalibration)g

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 可执行文件
add_executable(calibration_demo 
               main.cpp
               calibration.cpp)

# 链接OpenCV库
target_link_libraries(calibration_demo ${OpenCV_LIBS})

