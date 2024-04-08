cmake_minimum_required(VERSION 3.8)
project(consai_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(consai_msgs REQUIRED)
find_package(robocup_ssl_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/geometry/geometry_tools.py
    ${PROJECT_NAME}/hasher/constraint_hasher.py
    ${PROJECT_NAME}/hasher/robot_control_hasher.py
    DESTINATION lib/${PROJECT_NAME})


add_library(${PROJECT_NAME}_geometry_tools SHARED
  src/geometry_tools.cpp
)
target_include_directories(${PROJECT_NAME}_geometry_tools
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_geometry_tools
  consai_msgs
  robocup_ssl_msgs
)
install(TARGETS ${PROJECT_NAME}_geometry_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(${PROJECT_NAME}_geometry_tools)
ament_export_targets(${PROJECT_NAME}_geometry_tools HAS_LIBRARY_TARGET)
ament_export_dependencies(consai_msgs robocup_ssl_msgs)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_geometry_tools tests/test_geometry_tools.py)
  ament_add_pytest_test(test_constraint_hasher tests/test_constraint_hasher.py)
  ament_add_pytest_test(test_robot_control_hasher tests/test_robot_control_hasher.py)
endif()

ament_package()
