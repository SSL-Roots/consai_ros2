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


add_library(geometry_tools SHARED
  src/geometry_tools.cpp
)
target_include_directories(geometry_tools
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(geometry_tools
  consai_msgs
  robocup_ssl_msgs
)
install(DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS geometry_tools
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(consai_msgs robocup_ssl_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg
  ) 

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint()
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_geometry_tools tests/test_geometry_tools.py)
  ament_add_pytest_test(test_constraint_hasher tests/test_constraint_hasher.py)
  ament_add_pytest_test(test_robot_control_hasher tests/test_robot_control_hasher.py)

  find_package(ament_cmake_gtest)
  ament_add_gtest(geometry_tools_test
    tests/cpp/test_geometry_tools.cpp
  )
  target_link_libraries(geometry_tools_test
    geometry_tools)
endif()

ament_package()
