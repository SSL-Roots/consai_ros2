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
find_package(Protobuf REQUIRED)

# Build protobuf C++ lib
set(protobuf_files
  proto/grSim_Commands.proto
  proto/grSim_Packet.proto
  proto/grSim_Replacement.proto
  proto/grSim_Robotstatus.proto
  proto/messages_robocup_ssl_wrapper.proto
  proto/messages_robocup_ssl_geometry.proto
  proto/messages_robocup_ssl_detection.proto
  proto/ssl_gc_common.proto
  proto/ssl_gc_game_event.proto
  proto/ssl_gc_geometry.proto
  proto/ssl_gc_referee_message.proto
)
protobuf_generate_cpp(PROTO_CPP PROTO_H
  ${protobuf_files}
)
add_library(proto_cpp
  SHARED ${PROTO_CPP} ${PROTO_H}
)
target_link_libraries(proto_cpp ${PROTOBUF_LIBRARIES})

# Install
## For Foxy
install(
  FILES ${PROTO_H}
  DESTINATION include/${PROJECT_NAME})

## For Humble
install(
  FILES ${PROTO_H}
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

install(
  TARGETS proto_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# Export
ament_export_libraries(proto_cpp)
ament_export_dependencies(Protobuf)

# Build ROS msgs
set(msg_files
  "msg/detection/DetectionBall.msg"
  "msg/detection/DetectionFrame.msg"
  "msg/detection/DetectionRobot.msg"
  "msg/geometry/BallModelChipFixedLoss.msg"
  "msg/geometry/BallModelStraightTwoPhase.msg"
  "msg/geometry/FieldCircularArc.msg"
  "msg/geometry/FieldLineSegment.msg"
  "msg/geometry/GeometryCameraCalibration.msg"
  "msg/geometry/GeometryData.msg"
  "msg/geometry/GeometryFieldSize.msg"
  "msg/geometry/GeometryModels.msg"
  "msg/geometry/Vector2f.msg"
  "msg/referee/Point.msg"
  "msg/referee/Referee.msg"
  "msg/referee/TeamInfo.msg"
  "msg/grsim/BallReplacement.msg"
  "msg/grsim/Commands.msg"
  "msg/grsim/Replacement.msg"
  "msg/grsim/RobotCommand.msg"
  "msg/grsim/RobotReplacement.msg"
  "msg/grsim/RobotStatus.msg"
  "msg/grsim/RobotsStatus.msg"
  "msg/detection_tracked/KickedBall.msg"
  "msg/detection_tracked/RobotId.msg"
  "msg/detection_tracked/TrackedBall.msg"
  "msg/detection_tracked/TrackedFrame.msg"
  "msg/detection_tracked/TrackedRobot.msg"
  "msg/detection_tracked/Vector2.msg"
  "msg/detection_tracked/Vector3.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  # ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
