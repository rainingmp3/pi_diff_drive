cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mobile_base_hardware_interface.cpp
  src/wheel.cpp  
  src/arduino_comms.cpp
  src/pid.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  controller_manager
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  serial
  geometry_msgs
  nav_msgs
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  serial
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
)

ament_package()
