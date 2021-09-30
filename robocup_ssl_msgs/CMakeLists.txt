cmake_minimum_required(VERSION 3.5)
project(robocup_ssl_msgs)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/BallModelChipFixedLoss.msg"
  "msg/BallModelStraightTwoPhase.msg"
  "msg/DetectionBall.msg"
  "msg/DetectionFrame.msg"
  "msg/DetectionRobot.msg"
  "msg/FieldCircularArc.msg"
  "msg/FieldLineSegment.msg"
  "msg/GeometryCameraCalibration.msg"
  "msg/GeometryData.msg"
  "msg/GeometryFieldSize.msg"
  "msg/GeometryModels.msg"
  "msg/Vector2f.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
