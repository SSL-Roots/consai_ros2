cmake_minimum_required(VERSION 3.5)
project(consai_robot_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(consai_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

# Controller Component
add_library(controller_component SHARED
  src/controller_component.cpp
)
target_compile_definitions(controller_component
  PRIVATE "CONSAI_VISION_TRACKER_BUILDING_DLL")
ament_target_dependencies(controller_component
  consai_msgs
  rclcpp
  rclcpp_components
)
rclcpp_components_register_nodes(controller_component "consai_robot_controller::Controller")

# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(consai_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)

ament_export_include_directories(include)
ament_export_libraries(controller_component)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  controller_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
