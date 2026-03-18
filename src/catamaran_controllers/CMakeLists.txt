cmake_minimum_required(VERSION 3.8)
project(catamaran_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/body_force_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface
  catamaran_controllers_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES catamaran_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()