cmake_minimum_required(VERSION 3.11)
project(task1)

set(CMAKE_CXX_STANDARD 14)

add_executable(task1 main.cpp)

find_package(OpenCV REQUIRED)
target_link_libraries(task1 ${OpenCV_LIBS})