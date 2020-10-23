#cmake_minimum_required(VERSION 3.15)

project(real_process)

set(CMAKE_CXX_STANDARD 14)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(real_process main.cpp process.hpp)

target_link_libraries(real_process ${OpenCV_LIBS})

