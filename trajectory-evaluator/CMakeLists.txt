cmake_minimum_required(VERSION 3.15)
project(trajectory_evaluator)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()

add_executable(trajectory_evaluator src/main.cpp src/Pose.cpp src/Pose.h)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
