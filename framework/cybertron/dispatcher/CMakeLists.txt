project(cybertron_dispatcher)

aux_source_directory(${PROJECT_SOURCE_DIR} DISPATCHER_SRCS)
add_library(${PROJECT_NAME} ${DISPATCHER_SRCS})

install(TARGETS cybertron_dispatcher LIBRARY DESTINATION lib)
