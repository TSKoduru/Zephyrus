cmake_minimum_required(VERSION 3.5)
project(competitor_advanced_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Provided by px4-ros2-interface-lib @ release/1.16, built into the image's
# workspace. If this fails, the image predates WP0 -- rebuild it.
find_package(px4_ros2_cpp REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})

add_executable(skybeta_goto_mode src/main.cpp)
target_link_libraries(skybeta_goto_mode
  PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp)

install(TARGETS skybeta_goto_mode DESTINATION lib/${PROJECT_NAME})

ament_package()
