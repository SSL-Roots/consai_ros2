find_package(Protobuf REQUIRED)

include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
)
set(protobuf_files
  messages_robocup_ssl_wrapper.proto
  messages_robocup_ssl_geometry.proto
  messages_robocup_ssl_detection.proto
)

# C++
protobuf_generate_cpp(PROTO_CPP PROTO_H
  ${protobuf_files}
)
add_library(proto_cpp
  SHARED ${PROTO_CPP} ${PROTO_H}
)
target_link_libraries(proto_cpp ${PROTOBUF_LIBRARIES})

# Python
protobuf_generate_python(PROTO_PY
  ${protobuf_files}
)
add_custom_target(porto_py ALL DEPENDS ${PROTO_PY})

install(
  FILES ${PROTO_H}
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS proto_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(FILES ${PROTO_PY}
  DESTINATION "${PYTHON_INSTALL_DIR}")

ament_export_dependencies(Protobuf)
