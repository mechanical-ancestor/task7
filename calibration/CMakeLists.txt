# 指定cmake版本和项目名称
cmake_minimum_required(VERSION 3.10)      
project(task_7 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 优化运行性能 （可选的）
set(CMAKE_BUILD_TYPE Release)



# 设置海康SDK头文件和库文件目录路径
set(MVS_INCLUDE_DIR "/opt/MVS/include")
set(MVS_LIBRARY_DIR "/opt/MVS/lib/64")

find_package(OpenCV REQUIRED)
# 添加可执行文件，指定所有源文件
add_executable(task_7 
               calibration/camera_open_screenshot.cpp)



#  配置所有依赖的头文件目录
target_include_directories(task_7 PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
   
)

# 配置海康SDK库路径
target_link_directories(task_7 PRIVATE ${MVS_LIBRARY_DIR})

# 链接所有依赖库
target_link_libraries(task_7  PRIVATE
    ${OpenCV_LIBS}
    MvCameraControl   
    pthread          

)