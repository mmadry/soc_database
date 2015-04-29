cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SOC2PCLReader)

find_package(PCL 1.5.2 REQUIRED io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (read_syncpc src/read_syncpc.cpp src/SOC2PCLReader.cpp)
add_executable (read_crd src/read_crd.cpp src/SOC2PCLReader.cpp)

target_link_libraries(read_syncpc ${PCL_LIBRARIES} pcl_common pcl_io pcl_visualization)
target_link_libraries(read_crd ${PCL_LIBRARIES} pcl_common pcl_io pcl_visualization)