cmake_minimum_required(VERSION 3.8)
project(consai_game)


find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})
file(GLOB_RECURSE PYTHON_SCRIPTS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/*.py
)

message("Found Python scripts:")
foreach(SCRIPT ${PYTHON_SCRIPTS})
  message(" - ${SCRIPT}")
endforeach()

install(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS "shell_scripts/rosbag_record.bash"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  consai_game/play
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg
  ) 
endif()

ament_package()
