cmake_minimum_required(VERSION 3.1)

project(Detect)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}")

add_executable(Detect detect.cpp)

find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
	include_directories(${OPENCV_INCLUDE_DIR})
	target_link_libraries(Detect ${OpenCV_LIBS})
endif()