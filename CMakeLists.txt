project(histogram)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
set(CMAKE_CXX_FLAGS "-std=c++11")
add_executable(${PROJECT_NAME} ${SRC_LIST})

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries (histogram ${PCL_LIBRARIES})
