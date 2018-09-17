cmake_minimum_required(VERSION 2.8.3)
project(cybertron_topology)

aux_source_directory(communication ${PROJECT_NAME}_SRCS)
aux_source_directory(manager ${PROJECT_NAME}_SRCS)
aux_source_directory(role ${PROJECT_NAME}_SRCS)
aux_source_directory(. ${PROJECT_NAME}_SRCS)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

target_link_libraries(${PROJECT_NAME}
  protobuf
  cybertron_proto
  cybertron_transport
  fastrtps
  fastcdr
  -lrt)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)