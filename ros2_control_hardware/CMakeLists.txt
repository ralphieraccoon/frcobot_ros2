cmake_minimum_required(VERSION 3.16)
project(ros2_control_frcobot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ros2_control_frcobot
  SHARED
  frcobot_system.cpp
)
target_compile_features(ros2_control_frcobot PUBLIC cxx_std_17)
target_include_directories(ros2_control_frcobot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ros2_control_frcobot>
)
ament_target_dependencies(
  ros2_control_frcobot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_frcobot.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/ros2_control_frcobot
)
install(TARGETS ros2_control_frcobot
  EXPORT export_ros2_control_frcobot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_ros2_control_frcobot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()