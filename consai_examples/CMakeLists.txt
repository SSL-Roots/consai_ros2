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
    ${PROJECT_NAME}/ball_boy_test.py
    ${PROJECT_NAME}/control.py
    ${PROJECT_NAME}/control_by_referee.py
    ${PROJECT_NAME}/control_with_role.py
    ${PROJECT_NAME}/field_observer.py
    ${PROJECT_NAME}/game.py
    ${PROJECT_NAME}/operation.py
    ${PROJECT_NAME}/referee_parser.py
    ${PROJECT_NAME}/referee_visualize_parser.py
    ${PROJECT_NAME}/robot_operator.py
    ${PROJECT_NAME}/role_assignment.py
    ${PROJECT_NAME}/decisions/attacker.py
    ${PROJECT_NAME}/decisions/center_back1.py
    ${PROJECT_NAME}/decisions/center_back2.py
    ${PROJECT_NAME}/decisions/decision_base.py
    ${PROJECT_NAME}/decisions/goalie.py
    ${PROJECT_NAME}/decisions/side_back1.py
    ${PROJECT_NAME}/decisions/side_back2.py
    ${PROJECT_NAME}/decisions/sub_attacker.py
    ${PROJECT_NAME}/decisions/substitute.py
    ${PROJECT_NAME}/decisions/zone_defense.py
    ${PROJECT_NAME}/decisions/side_wing.py
    ${PROJECT_NAME}/operation_test/test_move_to.py
    ${PROJECT_NAME}/operation_test/pass_shoot_test.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_role_assignment tests/test_role_assignment.py)
  ament_add_pytest_test(test_robot_operator tests/test_robot_operator.py)
  ament_add_pytest_test(test_geometry_tools tests/test_geometry_tools.py)
  ament_add_pytest_test(test_field_observer tests/test_field_observer.py)
  ament_add_pytest_test(test_operation tests/test_operation.py)
endif()

ament_package()
