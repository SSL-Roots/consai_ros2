cmake_minimum_required(VERSION 3.5)
project(consai_msgs)

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
find_package(robocup_ssl_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ConstraintLine.msg"
  "msg/ConstraintObject.msg"
  "msg/ConstraintPose.msg"
  "msg/ConstraintTheta.msg"
  "msg/ConstraintXY.msg"
  "msg/GoalPose.msg"
  "msg/GoalPoses.msg"
  "msg/MotionCommand.msg"
  "msg/MotionCommandArray.msg"
  "msg/NamedTargets.msg"
  "msg/NaviOptions.msg"
  "msg/ParsedReferee.msg"
  "msg/RefereeSupportInfo.msg"
  "msg/RobotControlMsg.msg"
  "msg/RobotLocalVelocities.msg"
  "msg/RobotLocalVelocity.msg"
  "msg/State2D.msg"
)

set(action_files
  "action/RobotControl.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES std_msgs robocup_ssl_msgs
  # ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
