cmake_minimum_required(VERSION 3.8)

project(lidar_preprocessor)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# PCL
find_package(
  PCL REQUIRED
  COMPONENTS
    common
    filters
)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# PCL definitions
add_definitions(
  ${PCL_DEFINITIONS}
)

# =====================================================
# C++ NODE
# =====================================================

add_executable(
  pointcloud_preprocessor
  src/preprocessor.cpp
)

# ROS 2 dependencies
ament_target_dependencies(
  pointcloud_preprocessor

  rclcpp
  sensor_msgs
  pcl_conversions
)

# PCL libraries
target_link_libraries(
  pointcloud_preprocessor
  ${PCL_LIBRARIES}
)

# =====================================================
# INSTALL
# =====================================================

install(
  TARGETS
    pointcloud_preprocessor

  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/

  DESTINATION
    include
)

ament_package()