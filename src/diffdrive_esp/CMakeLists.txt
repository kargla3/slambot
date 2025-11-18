cmake_minimum_required(VERSION 3.16)
project(diffdrive_esp)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

# Create the hardware interface plugin
add_library(
  diffdrive_esp
  SHARED
  src/diffdrive_esp.cpp
)

target_compile_features(diffdrive_esp PUBLIC cxx_std_17)
target_include_directories(diffdrive_esp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diffdrive_esp>
)

ament_target_dependencies(
  diffdrive_esp PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_esp_plugin.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/diffdrive_esp
)

install(TARGETS diffdrive_esp
  EXPORT export_diffdrive_esp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES diffdrive_esp_plugin.xml
  DESTINATION share/diffdrive_esp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_diffdrive_esp HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle serial)

ament_package()