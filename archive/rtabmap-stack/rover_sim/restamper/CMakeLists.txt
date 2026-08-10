cmake_minimum_required(VERSION 3.16)
project(rover_sim_restamper)
set(CMAKE_CXX_STANDARD 17)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(restamper restamper.cpp)
target_link_libraries(restamper PUBLIC rclcpp::rclcpp
  ${sensor_msgs_TARGETS})
