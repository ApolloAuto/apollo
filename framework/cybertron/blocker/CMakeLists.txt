project(cybertron_blocker)

aux_source_directory(${PROJECT_SOURCE_DIR} BLOCKER_SRCS)
add_library(${PROJECT_NAME} ${BLOCKER_SRCS})

install(TARGETS cybertron_blocker LIBRARY DESTINATION lib)
