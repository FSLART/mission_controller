cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)


add_executable(mission_controller src/mission_controller.cpp)
ament_target_dependencies(mission_controller rclcpp std_msgs lart_msgs)

install(TARGETS
  mission_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
