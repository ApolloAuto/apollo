cmake_minimum_required(VERSION 2.8.3)
project(cybertron_transport)

aux_source_directory(common ${PROJECT_NAME}_SRCS)
aux_source_directory(dispatcher ${PROJECT_NAME}_SRCS)
aux_source_directory(message ${PROJECT_NAME}_SRCS)
aux_source_directory(qos ${PROJECT_NAME}_SRCS)
aux_source_directory(reactor ${PROJECT_NAME}_SRCS)
aux_source_directory(reader ${PROJECT_NAME}_SRCS)
aux_source_directory(rtps ${PROJECT_NAME}_SRCS)
aux_source_directory(shm ${PROJECT_NAME}_SRCS)
aux_source_directory(writer ${PROJECT_NAME}_SRCS)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

target_link_libraries(${PROJECT_NAME}
  protobuf
  cybertron_proto
  fastrtps
  fastcdr
  -lrt)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
