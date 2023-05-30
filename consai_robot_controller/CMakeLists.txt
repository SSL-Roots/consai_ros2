cmake_minimum_required(VERSION 3.5)
project(consai_robot_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(consai_frootspi_msgs REQUIRED)
find_package(consai_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robocup_ssl_msgs REQUIRED)

include_directories(include)

# Controller Component
add_library(controller_component SHARED
  src/controller_component.cpp
  src/field_info_parser.cpp
  src/geometry_tools.cpp
  src/control_tools.cpp
)
target_compile_definitions(controller_component
  PRIVATE "CONSAI_ROBOT_CONTROLLER_BUILDING_DLL")
ament_target_dependencies(controller_component
  consai_frootspi_msgs
  consai_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  robocup_ssl_msgs
)
rclcpp_components_register_nodes(controller_component "consai_robot_controller::Controller")

# GrSim Command Converter
add_library(grsim_command_converter SHARED
  src/grsim_command_converter.cpp
)
target_compile_definitions(grsim_command_converter
  PRIVATE "CONSAI_ROBOT_CONTROLLER_BUILDING_DLL")
ament_target_dependencies(grsim_command_converter
  consai_frootspi_msgs
  consai_msgs
  rclcpp
  rclcpp_components
  robocup_ssl_msgs
)
rclcpp_components_register_nodes(grsim_command_converter "consai_robot_controller::GrSimCommandConverter")

# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(consai_frootspi_msgs)
ament_export_dependencies(consai_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(robocup_ssl_msgs)

ament_export_include_directories(include)
ament_export_libraries(controller_component)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  controller_component
  grsim_command_converter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
  ament_add_gtest(control_tools_test test/control_tools_test.cpp src/control_tools.cpp)
endif()

ament_package()
