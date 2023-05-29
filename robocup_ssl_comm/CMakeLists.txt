cmake_minimum_required(VERSION 3.5)
project(robocup_ssl_comm)

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
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${robocup_ssl_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Vision Component
add_library(vision_component SHARED
  src/vision_component.cpp
)
target_compile_definitions(vision_component
  PRIVATE "ROBOCUP_SSL_COMM_BUILDING_DLL")
ament_target_dependencies(vision_component
  rclcpp
  rclcpp_components
  robocup_ssl_msgs
)
target_link_libraries(vision_component
  ${robocup_ssl_msgs_LIBRARIES} ${Boost_LIBRARIES}
)
rclcpp_components_register_nodes(vision_component "robocup_ssl_comm::Vision")

# Game Controller Component
add_library(game_controller_component SHARED
  src/game_controller_component.cpp
)
target_compile_definitions(game_controller_component
  PRIVATE "ROBOCUP_SSL_COMM_BUILDING_DLL")
ament_target_dependencies(game_controller_component
  rclcpp
  rclcpp_components
  robocup_ssl_msgs
)
target_link_libraries(game_controller_component
  ${robocup_ssl_msgs_LIBRARIES} ${Boost_LIBRARIES}
)
rclcpp_components_register_nodes(game_controller_component "robocup_ssl_comm::GameController")

# GrSim Component
add_library(grsim_component SHARED
  src/grsim_component.cpp
)
target_compile_definitions(grsim_component
  PRIVATE "ROBOCUP_SSL_COMM_BUILDING_DLL")
ament_target_dependencies(grsim_component
  rclcpp
  rclcpp_components
  robocup_ssl_msgs
)
target_link_libraries(grsim_component
  ${robocup_ssl_msgs_LIBRARIES} ${Boost_LIBRARIES}
)
rclcpp_components_register_nodes(grsim_component "robocup_ssl_comm::GrSim")

# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(robocup_ssl_msgs)

ament_export_include_directories(include)
ament_export_libraries(vision_component)
ament_export_libraries(game_controller_component)
ament_export_libraries(grsim_component)

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
  vision_component
  game_controller_component
  grsim_component
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
