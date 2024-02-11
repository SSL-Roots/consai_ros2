cmake_minimum_required(VERSION 3.8)
project(consai_visualizer_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Color.msg"
  "msg/Objects.msg"
  "msg/Point.msg"
  "msg/ShapeAnnotation.msg"
  "msg/ShapeArc.msg"
  "msg/ShapeCircle.msg"
  "msg/ShapeLine.msg"
  "msg/ShapePoint.msg"
  "msg/ShapeRectangle.msg"
  "msg/ShapeRobot.msg"
  "msg/ShapeTube.msg"
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
