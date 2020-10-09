cmake_minimum_required(VERSION 3.15)

project(task1)

set(CMAKE_CXX_STANDARD 14)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(task1 main.cpp)

target_link_libraries(task1 ${OpenCV_LIBS})

