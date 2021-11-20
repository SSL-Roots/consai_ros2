cmake_minimum_required(VERSION 3.5)
project(consai_examples)

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

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/control.py
    ${PROJECT_NAME}/control_by_referee.py
    ${PROJECT_NAME}/control_with_role.py
    ${PROJECT_NAME}/game.py
    ${PROJECT_NAME}/referee_parser.py
    ${PROJECT_NAME}/robot_operator.py
    ${PROJECT_NAME}/role_assignment.py
    ${PROJECT_NAME}/decisions/attacker.py
    ${PROJECT_NAME}/decisions/decision_base.py
    ${PROJECT_NAME}/decisions/goalie.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
