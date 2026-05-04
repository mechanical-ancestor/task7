cmake_minimum_required(VERSION 3.8)
project(auto_aim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(OPENVINO_ROOT "/opt/intel/openvino_2024.6.0")
set(OPENVINO_INCLUDE_DIR "${OPENVINO_ROOT}/runtime/include")
set(OPENVINO_LIB_DIR "${OPENVINO_ROOT}/runtime/lib/intel64")

set(MVCAMERACONTROL_ROOT "/opt/MVS/")
set(MVCAMERACONTROL_INCLUDE_DIR "${MVCAMERACONTROL_ROOT}/include")
set(MVCAMERACONTROL_LIB_DIR "${MVCAMERACONTROL_ROOT}/lib/64")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#普通编译执行 src/auto_aim 下的代码
add_executable(auto_aim_camera     #海康相机
               src/auto_aim/auto_aim_camera.cpp
               serial/serial.cpp
               camera/Hik_Camera.cpp
               tools/filter/KF.cpp
               tools/solverPnP/solverPnP.cpp
               tools/yolo/yolo8.cpp
               )

add_executable(auto_aim_MV        #迈德威视相机
               src/auto_aim/auto_aim_MV.cpp
               serial/serial.cpp
               camera/MV_Camera.cpp
               tools/filter/KF.cpp
               tools/solverPnP/solverPnP.cpp
               tools/yolo/yolo8.cpp
               )

add_executable(auto_aim_cp          #电脑摄像头
               src/auto_aim/auto_aim_cp.cpp
               serial/serial.cpp
               tools/filter/KF.cpp
               tools/solverPnP/solverPnP.cpp
               tools/yolo/yolo8.cpp
               )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#auto_aim_camera
target_include_directories(auto_aim_camera SYSTEM PRIVATE
    ${OPENVINO_INCLUDE_DIR}
    ${MVCAMERACONTROL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_directories(auto_aim_camera  PRIVATE
    ${OPENVINO_LIB_DIR}
    ${MVCAMERACONTROL_LIB_DIR}
)

target_link_libraries(auto_aim_camera
    ${OpenCV_LIBS}
    openvino
    MvCameraControl
    yaml-cpp
)

#auto_aim_MV
target_include_directories(auto_aim_MV SYSTEM PRIVATE
    ${OPENVINO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_directories(auto_aim_MV  PRIVATE
    ${OPENVINO_LIB_DIR}
)

target_link_libraries(auto_aim_MV
    ${OpenCV_LIBS}
    openvino
    MVSDK
    yaml-cpp
)

#auto_aim_cp
target_include_directories(auto_aim_cp SYSTEM PRIVATE
    ${OPENVINO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_directories(auto_aim_cp  PRIVATE
    ${OPENVINO_LIB_DIR}
)

target_link_libraries(auto_aim_cp
    ${OpenCV_LIBS}
    openvino
    yaml-cpp
)

#ros
add_executable(auto_aim_cp_node
               src/ros/auto_aim_cp.cpp
               serial/serial.cpp
               tools/filter/KF.cpp
               tools/solverPnP/solverPnP.cpp
               tools/yolo/yolo8.cpp
               )

ament_target_dependencies(auto_aim_cp_node rclcpp)
ament_target_dependencies(auto_aim_cp_node sensor_msgs)

target_include_directories(auto_aim_cp_node SYSTEM PRIVATE
    ${OPENVINO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_directories(auto_aim_cp_node  PRIVATE
    ${OPENVINO_LIB_DIR}
)

target_link_libraries(auto_aim_cp_node
    ${OpenCV_LIBS}
    openvino
    yaml-cpp
    cv_bridge::cv_bridge
)

add_executable(auto_aim_node
               src/ros/auto_aim.cpp
               serial/serial.cpp
               tools/filter/KF.cpp
               tools/solverPnP/solverPnP.cpp
               tools/yolo/yolo8.cpp
               )

ament_target_dependencies(auto_aim_node rclcpp)
ament_target_dependencies(auto_aim_node sensor_msgs)

target_include_directories(auto_aim_node SYSTEM PRIVATE
    ${OPENVINO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_directories(auto_aim_node  PRIVATE
    ${OPENVINO_LIB_DIR}
)

target_link_libraries(auto_aim_node
    ${OpenCV_LIBS}
    openvino
    yaml-cpp
    cv_bridge::cv_bridge
)

install(TARGETS
  auto_aim_cp_node
  auto_aim_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
