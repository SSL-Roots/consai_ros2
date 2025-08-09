cmake_minimum_required(VERSION 3.8)
project(consai_referee_parser)

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

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg
  ) 
endif()

ament_package()
