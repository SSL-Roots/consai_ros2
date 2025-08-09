cmake_minimum_required(VERSION 3.5)
project(consai_visualizer)

# Load ament and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR src/${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/consai_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg
  ) 
endif()

ament_package()