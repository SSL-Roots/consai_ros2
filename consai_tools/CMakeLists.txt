cmake_minimum_required(VERSION 3.8)
project(consai_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/geometry/geometry_tools.py
    ${PROJECT_NAME}/hasher/constraint_hasher.py
    ${PROJECT_NAME}/hasher/robot_control_hasher.py
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_geometry_tools tests/test_geometry_tools.py)
  ament_add_pytest_test(test_constraint_hasher tests/test_constraint_hasher.py)
  ament_add_pytest_test(test_robot_control_hasher tests/test_robot_control_hasher.py)
endif()

ament_package()
