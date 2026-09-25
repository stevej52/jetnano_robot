cmake_minimum_required(VERSION 3.8)
project(jetnano_watchdog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(topic_watch src/topic_watch.cpp)
target_compile_features(topic_watch PUBLIC cxx_std_17)
ament_target_dependencies(topic_watch rclcpp diagnostic_msgs)

install(TARGETS topic_watch DESTINATION lib/${PROJECT_NAME})

ament_package()
