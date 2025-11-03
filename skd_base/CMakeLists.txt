cmake_minimum_required(VERSION 3.8)
project(skd_base LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)

## Compile
add_library(
  ${PROJECT_NAME}
  SHARED
  src/skd_hardware.cpp
  src/serial_HDLC_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_interface
  rclcpp_lifecycle
  rclcpp
  # realtime_tools
  pluginlib
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface skd_base.xml)

## Install
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )

## Exports
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  controller_interface
  rclcpp_lifecycle
  rclcpp
#   realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
