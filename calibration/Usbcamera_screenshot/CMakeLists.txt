cmake_minimum_required(VERSION 3.10)
project(Usbcamera)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)

# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 生成可执行文件
add_executable(Usbcamera 1.cpp)

# 链接OpenCV库
target_link_libraries(Usbcamera ${OpenCV_LIBS})