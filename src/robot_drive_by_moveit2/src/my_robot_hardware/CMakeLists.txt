cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(
  my_robot_hardware
  src/zp25s_system.cpp
)
target_include_directories(my_robot_hardware PRIVATE include)
ament_target_dependencies(
  my_robot_hardware
  hardware_interface
  pluginlib
  rclcpp
  control_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(
  TARGETS my_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(my_robot_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp)

ament_package()
