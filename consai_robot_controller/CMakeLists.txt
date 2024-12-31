cmake_minimum_required(VERSION 3.11)
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
find_package(consai_tools REQUIRED)
find_package(consai_visualizer_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robocup_ssl_msgs REQUIRED)

include(FetchContent)
fetchcontent_declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
fetchcontent_makeavailable(json)

include_directories(include)

# Controller Component
add_library(controller_component SHARED
  src/controller_component.cpp
  src/field_info_parser.cpp
  src/visualization_data_handler.cpp
  src/detection_extractor.cpp
  src/constraint_parser.cpp
  src/obstacle/obstacle_observer.cpp
  src/tactic/back_dribble_tactics.cpp
  src/tactic/ball_boy_tactics.cpp
  src/tactic/dribble_tactics.cpp
  src/tactic/shoot_tactics.cpp
  src/tactic/tactic_control_ball.cpp
  src/tactic/tactic_obstacle_avoidance.cpp
  src/tools/control_tools.cpp
  src/bangbangtrajectory1d.cpp
  src/bangbangtrajectory2d.cpp
  src/bangbangtrajectory3d.cpp
    src/trajectory_follow_control.cpp
  src/trajectory_generator.cpp
  src/trajectory.cpp
  src/locomotion_controller.cpp
  src/utils.cpp
  src/global_for_debug.cpp
)
target_compile_definitions(controller_component
  PRIVATE "CONSAI_ROBOT_CONTROLLER_BUILDING_DLL")
ament_target_dependencies(controller_component
  consai_frootspi_msgs
  consai_msgs
  consai_tools
  consai_visualizer_msgs
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
target_link_libraries(grsim_command_converter nlohmann_json::nlohmann_json)
ament_target_dependencies(grsim_command_converter
  consai_frootspi_msgs
  consai_msgs
  consai_visualizer_msgs
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

# FetchContentで取得したnlohmann/jsonのターゲットもインストール
install(
  TARGETS nlohmann_json
  EXPORT export_${PROJECT_NAME}
)

# エクスポートセットをインストール
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
  ament_add_gtest(control_tools_test
    test/control_tools_test.cpp
    src/tools/control_tools.cpp
  )

  ament_add_gtest(obstacle_observer_test
    test/obstacle_observer_test.cpp
  )
  target_link_libraries(obstacle_observer_test controller_component)

  ament_add_gtest(obstacle_environment_test test/obstacle_environment_test.cpp)
endif()

ament_package()
