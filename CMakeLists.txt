cmake_minimum_required(VERSION 3.16)

project(task7_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio calib3d imgcodecs video)

add_library(task7_core
    src/auto_aim/detector.cpp
    src/auto_aim/solver.cpp
    src/auto_aim/tracker.cpp
    src/calibration/calibrator.cpp
    src/calibration/capture.cpp
    src/config/config.cpp
    src/io/camera.cpp
    src/io/serial.cpp
)

target_include_directories(task7_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(task7_core
    PUBLIC
        ${OpenCV_LIBS}
)

if(WIN32)
    target_link_libraries(task7_core PUBLIC setupapi)
endif()

add_executable(task7_main src/main.cpp)
target_link_libraries(task7_main PRIVATE task7_core)
