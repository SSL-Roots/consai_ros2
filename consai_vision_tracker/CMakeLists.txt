cmake_minimum_required(VERSION 3.5)
project(consai_vision_tracker)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robocup_ssl_msgs REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(
  include
  ${BFL_INCLUDE_DIRS}
)

# Tracker Component
add_library(tracker_component SHARED
  src/tracker_component.cpp
  src/ball_tracker.cpp
  src/robot_tracker.cpp
)
target_compile_definitions(tracker_component
  PRIVATE "CONSAI_VISION_TRACKER_BUILDING_DLL")
ament_target_dependencies(tracker_component
  rclcpp
  rclcpp_components
  robocup_ssl_msgs
)
target_link_libraries(tracker_component
  ${BFL_LIBRARIES}
)
rclcpp_components_register_nodes(tracker_component "consai_vision_tracker::Tracker")

# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(robocup_ssl_msgs)

ament_export_include_directories(include)
ament_export_libraries(tracker_component)

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
  tracker_component
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
