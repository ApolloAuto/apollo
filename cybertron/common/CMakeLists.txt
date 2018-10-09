project(cybertron_common)

aux_source_directory(${PROJECT_SOURCE_DIR} COMMON_SRCS)
add_library(${PROJECT_NAME} ${COMMON_SRCS})
target_link_libraries(${PROJECT_NAME}
  cybertron_proto
  ${PROTOBUF_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)
install(TARGETS cybertron_common LIBRARY DESTINATION lib)
