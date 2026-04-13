

cmake_minimum_required(VERSION 3.10)      
project(task_7 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 优化（可选的）
set(CMAKE_BUILD_TYPE Release)

# 海康SDK头文件和库文件目录路径
set(MVS_INCLUDE_DIR "/opt/MVS/include")
set(MVS_LIBRARY_DIR "/opt/MVS/lib/64")

# 找 yaml-cpp 依赖
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
# 指定所有源文件
add_executable(task_7 
               src/aim.cpp
               tasks/yolov8.cpp
               tasks/include/camera.cpp
               tasks/include/target.cpp
               tasks/include/Usbcamera.cpp
               io/serial.cpp
               )



# 3. 配置头文件目录
target_include_directories(task_7 PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}                # 项目根目录
    ${YAML_CPP_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/tasks  # 添加这一行，让编译器能找到 tasks/yolov8.hpp
    ${PROJECT_SOURCE_DIR}/tasks/include #同理
    ${PROJECT_SOURCE_DIR}/io
    ${PROJECT_SOURCE_DIR}/src
)

# 4. 配置海康SDK库路径
target_link_directories(task_7 PRIVATE ${MVS_LIBRARY_DIR})

# 5. 链接所有依赖库
target_link_libraries(task_7  PRIVATE
    ${OpenCV_LIBS}
    MvCameraControl   # 海康 MVS SDK 的核心库
    pthread           # Linux 下的线程库，支持多线程相关功能
    openvino::runtime  # 链接 OpenVINO 核心库
    yaml-cpp          # 链接 yaml-cpp 库
    util              # 支持termios串口配置函数
    rt                # 支持Linux实时操作，兼容串口文件描述符操作
)