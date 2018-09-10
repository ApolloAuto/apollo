project(cybertron_record)

add_library(${PROJECT_NAME}
  header_builder.cpp
  record_file.cpp
  record_writer.cpp
)

target_link_libraries(${PROJECT_NAME}
  cybertron_common
  ${PROTOBUF_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  bz2
  lz4
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
