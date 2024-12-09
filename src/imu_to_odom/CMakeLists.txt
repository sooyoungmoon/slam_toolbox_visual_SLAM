cmake_minimum_required(VERSION 3.5)
project(imu_to_odom)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

add_executable(myTfBroacaster src/my_tf2_broadcaster.cpp)
ament_target_dependencies(
  myTfBroacaster 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs
  eigen3_cmake_module
  )

install(TARGETS
  myTfBroacaster
  DESTINATION lib/${PROJECT_NAME})

ament_package()