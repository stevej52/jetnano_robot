cmake_minimum_required(VERSION 3.8)
project(jetnano_watchdog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(topic_watch src/topic_watch.cpp)
target_compile_features(topic_watch PUBLIC cxx_std_17)
ament_target_dependencies(topic_watch rclcpp diagnostic_msgs)

# tilt guard + motion check + VO watchdog in one process (2026-09-27; were ~19 % of a core in Python)
add_executable(safety_monitor src/safety_monitor.cpp)
target_compile_features(safety_monitor PUBLIC cxx_std_17)
ament_target_dependencies(safety_monitor rclcpp action_msgs geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros)

# the BNO055 driver (2026-09-30; bno055_lean.py was 9-11 % of a core at 100 Hz)
add_executable(bno055_imu src/bno055_imu.cpp)
target_compile_features(bno055_imu PUBLIC cxx_std_17)
ament_target_dependencies(bno055_imu rclcpp geometry_msgs sensor_msgs std_msgs)

# nvblox's grid as points for the guard, and onto the map for the planner (2026-09-30; was 4.5-8 % in Python)
add_executable(grid_to_points src/grid_to_points.cpp)
target_compile_features(grid_to_points PUBLIC cxx_std_17)
ament_target_dependencies(grid_to_points rclcpp nav_msgs sensor_msgs tf2_msgs)
target_link_libraries(grid_to_points ${OpenCV_LIBS})
target_include_directories(grid_to_points PRIVATE ${OpenCV_INCLUDE_DIRS})

install(TARGETS topic_watch safety_monitor bno055_imu grid_to_points DESTINATION lib/${PROJECT_NAME})

ament_package()
