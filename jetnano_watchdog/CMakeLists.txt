cmake_minimum_required(VERSION 3.8)
project(jetnano_watchdog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(topic_watch src/topic_watch.cpp)
target_compile_features(topic_watch PUBLIC cxx_std_17)
ament_target_dependencies(topic_watch rclcpp diagnostic_msgs)

# tilt guard + motion check + VO watchdog in one process (2026-09-27; were ~19 % of a core in Python)
add_executable(safety_monitor src/safety_monitor.cpp)
target_compile_features(safety_monitor PUBLIC cxx_std_17)
ament_target_dependencies(safety_monitor rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros)

install(TARGETS topic_watch safety_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()
